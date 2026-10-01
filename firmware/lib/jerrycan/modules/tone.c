/**
 * @file tone.c
 * @brief JerryCAN Tone Generator Message Handling
 *
 * This file provides functionality to manage and transmit tone generator information
 * via CAN messages through the JerryCAN library. It enables the transmission of tone
 * generator status messages, as well as handling incoming commands to play specific
 * tones with specified frequencies and durations.
 *
 * Timer Rate: CONFIG_LIB_JERRYCAN_TONE_TX_PERIOD_MS (in milliseconds, configurable via Kconfig)
 * - Sets the periodic rate for transmitting tone generator status messages.
 *
 * Key Functions:
 * - `jerrycan_tone_generator_tx()`: Constructs and transmits a CAN message for the
 *    given tone generator instance.
 * - `jerrycan_tone_generator_bulk_tx()`: Iterates through each tone generator instance,
 *    retrieves its frequency and remaining duration, and sends the information as a CAN message.
 * - `jerrycan_tone_generator_write_handler()`: Processes incoming CAN messages to
 *    play a tone with specified frequency and duration for a specific tone generator.
 * - `jerrycan_tone_init()`: Initializes the tone generator module, registers the CAN
 *    message handler, and starts the timer to send periodic status messages.
 *
 * Dependencies:
 * - `ll_tone_generator_get_frequency()`: Retrieves the current frequency of a tone generator.
 * - `ll_tone_generator_get_time_remaining()`: Retrieves the remaining time for the current tone.
 * - `ll_tone_generator_play_tone()`: Plays a tone with specified frequency and duration.
 * - `jerrycan_register_rx_callback()`: Registers a CAN message callback in the JerryCAN system.
 * - `jerrycan_tx()`: Transmits CAN messages via the JerryCAN library.
 *
 * Usage:
 * This module initializes at startup using Zephyr’s SYS_INIT macro. It starts a timer
 * to periodically transmit status messages at the rate specified by
 * `CONFIG_LIB_JERRYCAN_TONE_TX_PERIOD_MS`. The `jerrycan_tone_generator_write_handler()`
 * handles incoming CAN messages to control tone generation, setting the tone frequency
 * and duration as instructed.
 */

#include <zephyr/logging/log.h>

#include "jerrycan.h"
#include "tone_generator.h"

LOG_MODULE_DECLARE(jerrycan, CONFIG_LIB_JERRYCAN_LOG_LEVEL);

#define DT_DRV_COMPAT ll_tone_generator

/* Number of enabled tone generator instances found in the device tree */
#define TONE_GENERATOR_COUNT DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)

/* JerryCAN tone generator context structure */
typedef struct {
    uint16_t instance_number;
    const struct device *tone_generator;
    uint8_t uuid;
} jerrycan_tone_context_t;

#define TONE_CONTEXT(id) \
    (jerrycan_tone_context_t) { .instance_number = id, .tone_generator = DEVICE_DT_INST_GET(id), .uuid = 0 }

#define TONE_CONTEXT_COMMA(id) TONE_CONTEXT(id),

/* Array of tone generator contexts for all enabled tone generator devices */
static jerrycan_tone_context_t contexts[TONE_GENERATOR_COUNT] = {DT_INST_FOREACH_STATUS_OKAY(TONE_CONTEXT_COMMA)};

static void jerrycan_tone_generator_tx(jerrycan_tone_context_t *context) {
    const struct device *tone_generator = context->tone_generator;
    uint16_t instance_number = context->instance_number;

    jerrycan_msg_t msg = {.type = JERRYCAN_CMD_TONE,
                          .tone = {
                              .instance = instance_number,
                              .frequency_hz = (uint16_t)ll_tone_generator_get_frequency(tone_generator),
                              .duration_ms = ll_tone_generator_get_time_remaining(tone_generator),
                          }};

    int ret = jerrycan_tx(&msg, K_NO_WAIT);
    if (ret < 0) {
        LOG_WRN("Failed to transmit CAN message for tone_generator%d", instance_number);
    }

    if (msg.tone.duration_ms == 0 && context->uuid != 0) {
        jerrycan_send_ack(context->uuid, 0);
        context->uuid = 0;
    }
}

static void jerrycan_tone_generator_bulk_tx() {
    /* For each tone generator, construct and send a tone message */
    for (int i = 0; i < TONE_GENERATOR_COUNT; i++) {
        jerrycan_tone_generator_tx(&contexts[i]);
    }
}

static int jerrycan_tone_generator_write_handler(const jerrycan_msg_t *msg) {
    /* Pull parameters from jerrycan message */
    uint16_t instance = msg->tone.instance;
    uint16_t frequency_hz = msg->tone.frequency_hz;
    uint16_t duration_ms = msg->tone.duration_ms;

    LOG_INF("Recieved tone generator play tone message: instance=%d, frequency_hz=%d, duration_ms=%d uuid=%d", instance,
            frequency_hz, duration_ms, +msg->uuid);

    int idx = 0;
    while (idx < TONE_GENERATOR_COUNT && contexts[idx].instance_number != instance) {
        idx++;
    }

    int rc;
    if (idx >= TONE_GENERATOR_COUNT) {
        LOG_ERR("Failed to write tone over CAN: Invalid instance - %d", instance);
        rc = -ENOENT;
    } else {
        /* Grab tone generator instance */
        const struct device *tone_generator = contexts[idx].tone_generator;

        /* Play tone with the specified parameters, printing error on failure */
        rc = ll_tone_generator_play_tone(tone_generator, frequency_hz, duration_ms);
        if (rc != 0) {
            LOG_ERR("Failed to write tone over CAN: Error playing tone - %d", rc);
        } else {
            // Only once the duration timer runs: before that, the status timer would see no time remaining and ack
            // this command as complete, and a failed command is acked by the dispatcher instead.
            contexts[idx].uuid = msg->uuid;

            /* Transmit state immediately after starting tone */
            jerrycan_tone_generator_tx(&contexts[idx]);
        }
    }

    return rc == 0 ? COMMAND_NOT_COMPLETE : rc;
}

static jerrycan_rx_callback_t tone_callback = {
    .filter_msg_type = JERRYCAN_CMD_TONE,
    .func = jerrycan_tone_generator_write_handler,
};

K_TIMER_DEFINE(jerrycan_tone_generator_timer, jerrycan_tone_generator_bulk_tx, NULL);

static int jerrycan_tone_init() {
    jerrycan_register_rx_callback(&tone_callback);

    /* Start timer to send the status messages periodically */
    k_timer_start(&jerrycan_tone_generator_timer, K_MSEC(100), K_MSEC(CONFIG_LIB_JERRYCAN_TONE_TX_PERIOD_MS));

    return 0;
}

SYS_INIT(jerrycan_tone_init, APPLICATION, CONFIG_LIB_JERRYCAN_INIT_PRIORITY);
