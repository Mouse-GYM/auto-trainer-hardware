/**
 * @file status.c
 * @brief JerryCAN System Status Message Handling
 *
 * This file handles the periodic transmission of system status information via
 * the JerryCAN library. The status message provides key indicators about the
 * current state of the system, including emergency stop status, limit switch states,
 * button states, and motor statuses (stepper and servo).
 *
 * Timer Rate: CONFIG_LIB_JERRYCAN_STATUS_TX_PERIOD_MS (in milliseconds, configurable via Kconfig)
 * - Controls the frequency at which system status messages are sent over CAN.
 *
 * Key Functions:
 * - `jerrycan_status_tx()`: Constructs a status message with system indicators
 *    (e.g., estop, limit switches, buttons) and transmits it over CAN. This function
 *    currently uses placeholder values and should be updated to read actual status
 *    information from the hardware.
 * - `jerrycan_status_init()`: Initializes the status module and starts a timer to
 *    trigger `jerrycan_status_tx()` periodically.
 *
 * Dependencies:
 * - `jerrycan_tx()`: Transmits CAN messages via the JerryCAN library.
 *
 * Usage:
 * This module initializes at startup using Zephyr’s SYS_INIT macro. A periodic timer
 * is started to send status messages at the rate specified by `CONFIG_LIB_JERRYCAN_STATUS_TX_PERIOD_MS`.
 * The status message currently includes placeholder values, which should be updated
 * to reflect real-time system indicators from connected sensors and motors.
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "jerrycan.h"

LOG_MODULE_DECLARE(jerrycan, CONFIG_LIB_JERRYCAN_LOG_LEVEL);

void jerrycan_status_tx(struct k_timer *timer) {
    // Send a status message
    // FIXME: TODO: Actually read the status of the motors and sensors and populate this message
#if 0
    jerrycan_msg_t msg = {
        .type = JERRYCAN_CMD_STATUS,
        .status =
            {
                .estop_active = 0,
                .limit_switch0 = 0,
                .limit_switch1 = 0,
                .limit_switch2 = 0,
                .button0 = 0,
                .stepper_status0 = 0,
                .stepper_status1 = 0,
                .stepper_status2 = 0,
                .servo_status0 = 0,
                .servo_status1 = 0,
                .servo_status2 = 0,
            },
    };

    jerrycan_tx(&msg, K_NO_WAIT);
#endif
}

K_TIMER_DEFINE(jerrycan_status_timer, jerrycan_status_tx, NULL);

static struct _delay {
    uint8_t uuid;
} delay_data;

static void status_delay_expired(struct k_timer *timer) {
    jerrycan_send_ack(delay_data.uuid, 0);
    delay_data.uuid = 0;
}

K_TIMER_DEFINE(delay_timer, status_delay_expired, NULL);

static int status_delay(const jerrycan_msg_t *msg) {
    if (delay_data.uuid) {
        return -EBUSY;
    }
    delay_data.uuid = msg->uuid;
    k_timer_start(&delay_timer, K_MSEC(msg->delay.delay), K_MSEC(0));

    return COMMAND_NOT_COMPLETE;
}

static jerrycan_rx_callback_t delay_callback = {
    .filter_msg_type = JERRYCAN_CMD_DELAY,
    .func = status_delay,
};

static int jerrycan_status_init() {
    jerrycan_register_rx_callback(&delay_callback);

    // Start the timer that will send the status message periodically
    k_timer_start(&jerrycan_status_timer, K_MSEC(100), K_MSEC(CONFIG_LIB_JERRYCAN_STATUS_TX_PERIOD_MS));
    return 0;
}

SYS_INIT(jerrycan_status_init, APPLICATION, CONFIG_LIB_JERRYCAN_INIT_PRIORITY);
