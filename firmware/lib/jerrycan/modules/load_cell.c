/**
 * @file load_cell.c
 * @brief JerryCAN Load Cell Message Handling
 *
 * This file provides functionality to manage and transmit load cell information
 * via CAN messages through the JerryCAN library. Each load cell instance is
 * periodically read, and its load measurement is sent as a CAN message. This
 * provides continuous monitoring of real-time load cell data.
 *
 * Timer Rate: CONFIG_LIB_JERRYCAN_LOAD_CELL_TX_PERIOD_MS (in milliseconds, configurable via Kconfig)
 * - Determines the rate at which load cell data is sent over CAN.
 *
 * Key Functions:
 * - `jerrycan_load_cell_tx()`: Reads the current load measurement from each enabled
 *    load cell device and sends the data as a CAN message.
 * - `jerrycan_load_cell_init()`: Initializes the load cell module and starts the
 *    periodic timer to trigger `jerrycan_load_cell_tx()` at the configured interval.
 *
 * Dependencies:
 * - `ll_load_cell_get_load_mv()`: Retrieves the load measurement value in millivolts
 *    for each load cell.
 * - `jerrycan_tx()`: Transmits CAN messages via the JerryCAN library.
 *
 * Usage:
 * This module is automatically initialized at startup using Zephyr's SYS_INIT macro.
 * It starts a timer to periodically send load cell data over CAN at the rate specified
 * by `CONFIG_LIB_JERRYCAN_LOAD_CELL_TX_PERIOD_MS` in the Kconfig. Each enabled load cell
 * device instance is read, and its load value is transmitted as part of a CAN message.
 */

#include "load_cell.h"

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "jerrycan.h"

LOG_MODULE_DECLARE(jerrycan, CONFIG_LIB_JERRYCAN_LOG_LEVEL);

#define DT_DRV_COMPAT ll_load_cell

/* Number of enabled load cells found in the device tree */
#define LOAD_CELL_COUNT DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)

/* JerryCAN load cell context structure */
typedef struct {
    uint16_t instance_number;
    const struct device *load_cell;
} jerrycan_load_cell_context_t;

#define JERRYCAN_LOAD_CELL_CONTEXT(id) \
    (jerrycan_load_cell_context_t) { .instance_number = id, .load_cell = DEVICE_DT_INST_GET(id), }

#define JERRYCAN_LOAD_CELL_CONTEXT_COMMA(id) JERRYCAN_LOAD_CELL_CONTEXT(id),

/* Array of load cell contexts for all enabled load cell devices */
static const jerrycan_load_cell_context_t contexts[LOAD_CELL_COUNT] = {
    DT_INST_FOREACH_STATUS_OKAY(JERRYCAN_LOAD_CELL_CONTEXT_COMMA)};

static void jerrycan_load_cell_tx() {
    /* For each load cell, construct and send a load cell read message*/
    for (int i = 0; i < LOAD_CELL_COUNT; i++) {
        const jerrycan_load_cell_context_t *context = &contexts[i];
        const struct device *load_cell = context->load_cell;
        const uint16_t instance_number = context->instance_number;
        const float sample = ll_load_cell_get_load_mv_float(load_cell);

        jerrycan_msg_t msg = {.type = JERRYCAN_CMD_LOAD_CELL_READ,
                              .load_cell_read = {
                                  .instance = instance_number,
                                  .load_mv = sample,
                              }};

        jerrycan_tx(&msg, K_NO_WAIT);
    }
}

static int jerrycan_load_cell_tare_handler(const jerrycan_msg_t *msg) {
    int rc = -ENOENT;

    for (int i = 0; i < LOAD_CELL_COUNT; i++) {
        const jerrycan_load_cell_context_t *context = &contexts[i];
        const struct device *load_cell = context->load_cell;
        uint16_t instance_number = context->instance_number;

        if (msg->load_cell_tare.instance == instance_number) {
            rc = ll_load_cell_tare(load_cell);
            if (rc < 0) {
                LOG_ERR("Failed to perform the requested load cell tare operation: %d", rc);
            } else {
                LOG_INF("Successfully tared load_cell%d", instance_number);
                rc = 0;  // ensure we return rc == 0 for the uuid ack error field.
            }
            break;
        }
    }

    return rc;
}

static jerrycan_rx_callback_t load_cell_tare_callback = {
    .filter_msg_type = JERRYCAN_CMD_LOAD_CELL_TARE,
    .func = jerrycan_load_cell_tare_handler,
};

K_TIMER_DEFINE(jerrycan_load_cell_tx_timer, jerrycan_load_cell_tx, NULL);

static int jerrycan_load_cell_init() {
    jerrycan_register_rx_callback(&load_cell_tare_callback);

    /* Start timer to send the status messages periodically */
    k_timer_start(&jerrycan_load_cell_tx_timer, K_MSEC(100), K_MSEC(CONFIG_LIB_JERRYCAN_LOAD_CELL_TX_PERIOD_MS));

    return 0;
}

SYS_INIT(jerrycan_load_cell_init, APPLICATION, CONFIG_LIB_JERRYCAN_INIT_PRIORITY);
