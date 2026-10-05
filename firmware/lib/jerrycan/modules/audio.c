/**
 * Module to handle Audio (Microphone) input, calculate FFT frequency and
 * magnitude on the sampled data, and transmit the data on the CAN bus to the
 * host controller.
 *
 * Data collection and processing is performed in its own thread. The thread
 * yields control when it is waiting for new data.
 *
 * Data is collected at all times, as the data from the microphone is streaming.
 * Each data set is AUDIO_FFT_SIZE elements long.
 * Data is processed and transmitted on the CAN bus at AUDIO_UPDATE_RATE.
 *
 * When transmitting the data on the CAN bus, it sends the data set in a series
 * of CAN messages:
 * + The first message has an ID of JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_START;
 *   its payload is a unique packet number.
 * + The next set of messages has an ID of JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_CONT;
 *   its payload is a set of 2 frequency magnitude numbers as IEEE 32-bit floating
 *   point number.
 * + The last message has an ID of JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_END;
 *   its payload is the same unique packet number in the _START message.
 *
 * The receiving entity should accumulate the packets, ensuring that the _START
 * and _END packets have the same packet number, and that the correct number of
 * _CONT messages are also received. The protocol assumes there is no out-of-
 * order transmissions, but that there could be dropped packets.
 */

#include <math.h>
#include <sys/param.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "fft.h"
#include "generic_gpios.h"
#include "jerrycan.h"
#include "microphone.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define FFT_INPUT_SAMPLE_COUNT DT_PROP(DT_N_NODELABEL_microphone, block_size)
#define FFT_OUTPUT_FREQ_COUNT (FFT_INPUT_SAMPLE_COUNT / 2 + 1)  // DC, Freq, and Nyquest
#define AUDIO_UPDATE_RATE 10                                    // (Hz)
#define AUDIO_UPDATE_PERIOD (1000 / AUDIO_UPDATE_RATE)          // msec

LOG_MODULE_REGISTER(audio_in, CONFIG_LIB_JERRYCAN_LOG_LEVEL);

/**
 * Process the sampled audio data, transmitting it on the CAN bus when it's
 * done.
 *
 * @param packetNumber
 * @param rawData - Can NOT be NULL.
 */
static bool audio_process_data(uint64_t packetNumber, const int32_t* rawData);

/**
 * Determine if the data is good. The DMA/I2S transmissions are prone to dropping
 * words. This method checks to see if the incoming data is valid or if there
 * were dropped words in the formation of the data set.
 *
 * @param data Raw data from PCM1822
 * @param length Number of data points in data
 */
static bool validate_data(const int32_t* data, const size_t length);

/**
 * Translate the raw data (32-bit integers) to a complex data set, where the
 * imaginary potion is 0.
 *
 * @param rawData - Can NOT be NULL
 * @param[out] fftBuffer - This buffer must be 2x larger than the rawData buffer. Can NOT be NULL
 * @param length - Number of elements in rawData buffer
 */
static void audio_populate_complex_vector(const int32_t* rawData, float* fftBuffer, size_t length);

/**
 * Convert from magnitude unitless to magnitude dB
 * @param mag Vector of magnitude data
 * @param length Length of mag vector
 */
static void convert_to_db(float* mag, int length);

/**
 * Transmit the given payload (N bytes; where N is defined by the CAN payload
 * size) on the CAN bus.
 *
 * @param payload - Can NOT be NULL
 * @param length - Length of payload to transfer. Must be <= sizeof(jerrycan_cmd_audio_data_t).
 */
static void audio_transmit_data(const void* payload, size_t length);

/**
 * Report the calculated magnitude data on the CAN bus.
 *
 * @param packetNumber - Packet number
 * @param magnitude - Magnitude data. Can NOT be NULL
 * @param length - Number of elements in the magnitude vector.
 */
static void audio_report_data(uint64_t packetNumber, const float* magnitude, size_t length);

/**
 * Thread method.
 *
 * + Initialize the system
 * + Wait for data from the microphone
 * + At AUDIO_UPDATE_RATE, process and transmit magnitude data on the CAN bus
 *
 */
static void audio_thread(void*, void*, void*);

/* -------------------------------------------------------------------------- */

float g_windowing_map[FFT_INPUT_SAMPLE_COUNT];

static void populate_windowing_map() {
    for (int i = 0; i < FFT_INPUT_SAMPLE_COUNT; ++i) {
        // Hann window: 0.5 * (1 - cos(2π*i/(N-1)))
        g_windowing_map[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (FFT_INPUT_SAMPLE_COUNT - 1)));
    }
}

/* -------------------------------------------------------------------------- */

static void audio_thread(void*, void*, void*) {
    struct k_timer timer;
    const struct device* microphone = DEVICE_DT_GET_ANY(ll_microphone);

    populate_windowing_map();

    if (!microphone) {
        LOG_ERR("Microphone not defined in the DTS");
        return;
    }

    LOG_INF("THREAD: AudioIn:: STARTED.");

    // Set up periodic timer
    k_timer_init(&timer, NULL, NULL);
    k_timer_start(&timer, K_MSEC(1000), K_MSEC(AUDIO_UPDATE_PERIOD));

    if (ll_microphone_enable_reads(microphone, true)) {
        uint64_t packetNumber = 0;
        bool failed_last = false;

        while (true) {
            void* rawData;
            uint32_t sample_count;  // total samples across all channels

            const int rc = ll_microphone_read(microphone, &rawData, &sample_count);

            if (!rc) {
                // status_get returns the # of times timer expired since last call
                if (k_timer_status_get(&timer) > 0 || failed_last) {
                    failed_last = !audio_process_data(++packetNumber, rawData);
                }

                ll_microphone_release_buffer(microphone, rawData);
            } else if (rc == -EIO) {
                int ret = ll_microphone_reset_read(microphone);
                if (ret != 0) {
                    LOG_ERR("Microphone Reset Read Failed. Bailing");
                    break;
                }
            } else if (rc != -EAGAIN) {
                LOG_ERR("Microphone Read Failed. Bailing");
                break;
            }
        }
    }

    LOG_INF("THREAD: AudioIn:: STOPPED.");

    k_timer_stop(&timer);
    (void)ll_microphone_enable_reads(microphone, false);
}

K_THREAD_DEFINE(gThread, CONFIG_LIB_JERRYCAN_AUDIO_STACK_SIZE, audio_thread, NULL, NULL, NULL,
                CONFIG_LIB_JERRYCAN_AUDIO_PRIORITY, 0, 1000);

/* -------------------------------------------------------------------------- */

static bool audio_process_data(const uint64_t packetNumber, const int32_t* rawData) {
    Fft fft;

    if (validate_data(rawData, FFT_INPUT_SAMPLE_COUNT) && fft_initialize(&fft, FFT_INPUT_SAMPLE_COUNT)) {
        static float fftBuffer[FFT_INPUT_SAMPLE_COUNT * 2];  // Complex data set
        static float magnitude[FFT_OUTPUT_FREQ_COUNT];

        audio_populate_complex_vector(rawData, fftBuffer, FFT_INPUT_SAMPLE_COUNT);
        fft_calculate_frequency(&fft, fftBuffer);
        fft_calculate_magnitude(&fft, fftBuffer, magnitude);
        convert_to_db(magnitude, FFT_OUTPUT_FREQ_COUNT);
        audio_report_data(packetNumber, magnitude, FFT_OUTPUT_FREQ_COUNT);

        return true;
    }

    return false;
}

/* -------------------------------------------------------------------------- */

bool validate_data(const int32_t* data, const size_t length) {
    // Calculate mean
    float mean1 = 0.0f;
    float mean2 = 0.0f;

    for (size_t i = 0; i < length; i++) {
        mean1 += (float)data[i * 2];
        mean2 += (float)data[i * 2 + 1];
    }

    mean1 = mean1 / length;
    mean2 = mean2 / length;

    // Calculate standard deviation
    float vsum1 = 0.0f;
    float vsum2 = 0.0f;

    for (size_t i = 0; i < length; i++) {
        float diff;

        // Accumulate squared differences from mean for variance
        diff = (float)data[i * 2] - mean1;
        vsum1 += diff * diff;
        diff = (float)data[i * 2 + 1] - mean2;
        vsum2 += diff * diff;
    }

    const float stddev1 = sqrtf(vsum1 / length);
    const float stddev2 = sqrtf(vsum2 / length);

    return (fabs(mean1) < 1e5 || fabs(mean2) < 1e5) && (fabs(stddev1) < 1e5 || fabs(stddev2) < 1e5);
}

/* -------------------------------------------------------------------------- */

static void audio_populate_complex_vector(const int32_t* rawData, float* fftBuffer, const size_t length) {
    const struct device* microphone = DEVICE_DT_GET_ANY(ll_microphone);
    const int channel_count = ll_microphone_channel_count(microphone);

    for (size_t i = 0; i < length; i++) {
        int32_t raw = rawData[i * channel_count];

        // The DMA engine on the STM32 transfers 2 16-bit values from the I2S data
        // register. Frequently the two values are word-swapped or dropped.
        // Sometimes the first/second channels are also swapped.
        // Since there is an AC-bias on the floating channel #2 inputs, we can add with
        // impunity.
        raw += rawData[i * channel_count + 1];

        // Apply windowing to remove cross-frequecy bleed
        fftBuffer[i * 2] = (float)raw * g_windowing_map[i];

        // Complex portion is 0
        fftBuffer[i * 2 + 1] = 0.0f;
    }
}

/* -------------------------------------------------------------------------- */

static void convert_to_db(float* mag, int length) {
    for (int i = 0; i < length; ++i, ++mag) {
        const float epsilon = 1e-10f;  // Adding small epsilon (1e-10) to avoid log(0)
        *mag = 20 * log10f(*mag + epsilon);
    }
}

/* -------------------------------------------------------------------------- */

static void audio_report_data(uint64_t packetNumber, const float* magnitude, size_t length) {
    const int ELEMENTS_PER_MESSAGE = sizeof(jerrycan_cmd_audio_data_t) / sizeof(float);

    ++packetNumber;

    {
        jerrycan_msg_t msg;

        msg.type = JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_BEGIN;
        msg.audio_data_cmd.stream_id = packetNumber;

        jerrycan_tx(&msg, K_NO_WAIT);  // dont' wait; OK if a data sets drops
    }

    // Skip over DC (assumption, for the moment)
    for (int k = 1; k < length; k += ELEMENTS_PER_MESSAGE) {
        audio_transmit_data(magnitude, MIN(length - ELEMENTS_PER_MESSAGE, ELEMENTS_PER_MESSAGE) * sizeof(float));
        magnitude += ELEMENTS_PER_MESSAGE;
    }

    {
        jerrycan_msg_t msg;

        msg.type = JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_END;
        msg.audio_data_cmd.stream_id = packetNumber;

        jerrycan_tx(&msg, K_NO_WAIT);  // dont' wait; OK if a data sets drops
    }
}

/* -------------------------------------------------------------------------- */

static void audio_transmit_data(const void* payload, const size_t length) {
    jerrycan_msg_t msg;

    msg.type = JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_CONT;
    memcpy(msg.audio_data.payload, payload, length);

    jerrycan_tx(&msg, K_NO_WAIT);  // dont' wait; OK if a data sets drops
}
