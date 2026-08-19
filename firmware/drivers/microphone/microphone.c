
#include "microphone.h"

#include <assert.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT ll_microphone

LOG_MODULE_REGISTER(microphone, CONFIG_LIB_MIC_LOG_LEVEL);

#define INTEGRAL_TYPE int32_t
#define BYTES_PER_SAMPLE sizeof(INTEGRAL_TYPE)
#define SAMPLE_BIT_WIDTH (BYTES_PER_SAMPLE * CHAR_BIT)

#define CHANNEL_COUNT(idx) DT_INST_PROP(idx, channel_count)

// Block size is the number of total samples over all channels
#define BLOCK_SIZE(idx) (BYTES_PER_SAMPLE * DT_INST_PROP(idx, block_size) * CHANNEL_COUNT(idx))
#define BLOCK_COUNT 4  // one active, one processing, two on-deck

/**
 * Microphone data set
 */
typedef struct _data {
    bool initialized;
    bool streamEnabled;
    const struct device* i2sDevice;
} microphone_data_t;

typedef struct _config {
    struct i2s_config i2s_cfg;
} microphone_cfg_t;

/* -------------------------------------------------------------------------- */

/**
 * Initialize the microphone, configuring the device to receive data on the I2S
 * bus.
 *
 * @param device - Uninitialized Microphone instance.
 *
 * @retval 0 - Success
 * @retval <0 - Error code (negative errno)
 */
static int ll_microphone_initialize(const struct device* device) {
    const microphone_cfg_t* config = device->config;
    microphone_data_t* microphone = device->data;

    if (!microphone->i2sDevice) {
        LOG_ERR("No parent I2S Device!");
        return -ENOENT;
    }

    const int ret = i2s_configure(microphone->i2sDevice, I2S_DIR_RX, &(config->i2s_cfg));
    if (ret < 0) {
        LOG_ERR("Failed to configure Microphone input stream: %d\n", ret);
        return ret;
    }

    LOG_INF("Frequency = %d", config->i2s_cfg.frame_clk_freq);
    LOG_INF("Block Size = %d", config->i2s_cfg.block_size);

    LOG_INF("Initialized\n");

    microphone->initialized = true;

    return 0;
}

/* -------------------------------------------------------------------------- */

bool ll_microphone_enable_reads(const struct device* device, const bool enable) {
    int ret;
    microphone_data_t* mic = device->data;

    if (!mic->initialized) {
        static bool reported = false;
        if (!reported) {
            LOG_ERR("Must initialize the microphone entity before calling mic_enable_reads");
            reported = true;
        }
        return false;
    }

    if (enable) {
        ret = i2s_trigger(mic->i2sDevice, I2S_DIR_RX, I2S_TRIGGER_PREPARE);
        if (ret < 0) {
            LOG_ERR("Failed to prepare streaming: %d", ret);
            mic->streamEnabled = false;
            return false;
        }
    }

    ret = i2s_trigger(mic->i2sDevice, I2S_DIR_RX, enable ? I2S_TRIGGER_START : I2S_TRIGGER_STOP);
    if (ret < 0) {
        LOG_ERR("Failed to %sable streaming: %d", enable ? "en" : "dis", ret);
        mic->streamEnabled = false;
        return false;
    }

    mic->streamEnabled = enable;

    LOG_INF("Streaming %sabled\n", enable ? "en" : "dis");

    return true;
}

/* -------------------------------------------------------------------------- */

int ll_microphone_read(const struct device* device, void** mem_block, uint32_t* block_size) {
    const microphone_data_t* mic = device->data;

    // Make sure user doesn't use old or invalid data
    *mem_block = NULL;
    *block_size = 0;

    if (!mic->initialized) {
        static bool reported = false;
        if (!reported) {
            LOG_ERR("Must initialize the microphone entity before calling mic_read");
            reported = true;
        }
        return -ENOENT;
    }

    if (!mic->streamEnabled) {
        static bool reported = false;
        if (!reported) {
            LOG_ERR("Must enable reads before calling mic_read");
            reported = true;
        }

        return -EIO;
    }

    const int rc = i2s_read(mic->i2sDevice, mem_block, block_size);
    if (!rc) {
        *block_size = *block_size / BYTES_PER_SAMPLE;
    } else {
        *block_size = 0;
    }

    return rc;
}

/* -------------------------------------------------------------------------- */

void ll_microphone_release_buffer(const struct device* device, void* mem_block) {
    const microphone_cfg_t* config = device->config;

    k_mem_slab_free(config->i2s_cfg.mem_slab, mem_block);
}

/* -------------------------------------------------------------------------- */

int ll_microphone_channel_count(const struct device* device) {
    const microphone_cfg_t* config = device->config;

    return config->i2s_cfg.channels;
}

/* -------------------------------------------------------------------------- */

int ll_microphone_reset_read(const struct device* device) {
    // search for "zephyr i2s_read fails + recovery"
    const microphone_data_t* mic = device->data;
    const struct device* i2s_dev = mic->i2sDevice;
    const microphone_cfg_t* config = device->config;
    void* mem_block = NULL;
    uint32_t block_size;
    int ret;
    // put port/stream to stop->drop:
    ret = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_STOP);
    if (ret < 0) {
        LOG_ERR("Microphone Stop Failed. Bailing. ret=%d", ret);
        return ret;
    }
    ret = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_DROP);
    if (ret < 0) {
        LOG_ERR("Microphone Drop Failed. Bailing. ret=%d", ret);
        return ret;
    }
    // flush until EIO:
    if (false) {  // unsure about this one
        do {
            ret = i2s_read(i2s_dev, &mem_block, &block_size);
            if (ret == -EIO) {
                break;
            } else if (ret < 0) {
                LOG_ERR("Error purging microphone buffer. Bailing. ret=%d", ret);
                return ret;
            }
            k_mem_slab_free(config->i2s_cfg.mem_slab, mem_block);
        } while (true);
    }
    // now re-prepare and start the stream:
    ret = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_PREPARE);
    if (ret < 0) {
        LOG_ERR("Microphone Prepare Failed. Bailing. ret=%d", ret);
        return ret;
    }
    ret = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
    if (ret < 0) {
        LOG_ERR("Microphone Start Failed. Bailing. ret=%d", ret);
        return ret;
    }
    return 0;
}

/* -------------------------------------------------------------------------- */

#define INSTANCE_GENERATOR(idx)                                                                                   \
                                                                                                                  \
    K_MEM_SLAB_DEFINE_STATIC(mem_slab##idx, BLOCK_SIZE(idx) * CHANNEL_COUNT(idx), BLOCK_COUNT, BYTES_PER_SAMPLE); \
                                                                                                                  \
    static const microphone_cfg_t config##idx = {                                                                 \
        .i2s_cfg = {.word_size = SAMPLE_BIT_WIDTH,                                                                \
                    .channels = CHANNEL_COUNT(idx),                                                               \
                    .format = I2S_FMT_DATA_FORMAT_I2S,                                                            \
                    .options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,                                 \
                    .frame_clk_freq = DT_INST_PROP(idx, sampling_frequency),                                      \
                    .mem_slab = &mem_slab##idx,                                                                   \
                    .block_size = BLOCK_SIZE(idx),                                                                \
                    .timeout = K_TICKS_FOREVER},                                                                  \
    };                                                                                                            \
                                                                                                                  \
    static microphone_data_t data##idx = {                                                                        \
        .initialized = false, .streamEnabled = false, .i2sDevice = DEVICE_DT_GET(DT_INST_PARENT(idx))};           \
                                                                                                                  \
    DEVICE_DT_INST_DEFINE(idx, &ll_microphone_initialize, NULL, &data##idx, &config##idx, POST_KERNEL, 99, NULL);

DT_INST_FOREACH_STATUS_OKAY(INSTANCE_GENERATOR)
