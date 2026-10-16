/**
 * @file servo.c
 * @brief JerryCAN Servo Motor Message Handling
 *
 * This file handles the reception and processing of servo motor control messages
 * recieved via the JerryCAN library. It provides functionality to move the servo,
 * as well as to read and write servo configuration settings over CAN. The module
 * integrates with the motor library for controlling and configuring servo movements.
 *
 * Key Functions:
 * - `servo_handler()`: Processes incoming servo movement commands and logs the
 *    motion parameters. (Integration with the motor library is pending)
 * - `servo_cfg_write_handler()`: Handles configuration write messages for setting
 *    servo parameters, such as minimum, middle, and maximum positions.
 * - `servo_cfg_read_handler()`: Responds to configuration read requests by sending
 *    the current servo configuration parameters back over CAN. (Actual config read
 *    from the motor library is pending)
 * - `jerrycan_servo_init()`: Registers the above message handlers and initializes
 *    the servo handling module.
 *
 * Dependencies:
 * - `jerrycan_register_rx_callback()`: Registers a CAN message callback in the JerryCAN system.
 * - `jerrycan_tx()`: Transmits CAN messages via the JerryCAN library.
 *
 * Usage:
 * This module is initialized at startup using Zephyr’s SYS_INIT macro. It registers
 * callbacks to handle servo control messages, configuration write requests, and
 * configuration read requests. Actual servo control and configuration functionality
 * will be fully realized upon integration with the high-level motor library.
 */

#include <zephyr/logging/log.h>

#include "jerrycan.h"
#include "motor_common.h"
#include "motor_motion.h"
#include "motor_motion_workq.h"

LOG_MODULE_DECLARE(jerrycan, CONFIG_LIB_JERRYCAN_LOG_LEVEL);

#define DT_DRV_COMPAT ll_servo

/* Number of enabled servos found in the device tree */
#define SERVO_COUNT DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)

#define CAN_TIMEOUT K_MSEC(100)

// Per servo: a move was started and its completion is still to be acknowledged, with the context's `uuid`.
// Commands and the status work both run in cooperative threads (main and the system workqueue), so they don't
// preempt each other; this is only shared between them.
static bool ack_pending[SERVO_COUNT];

/**
 * Acknowledge servo `motor_id`'s finished move, if it has one, and return it to MOTION_IDLE.
 */
static void servo_consume_completion(const int motor_id) {
    struct servo_work_context *context = find_servo_context_from_device(servo_motor_by_id(motor_id));
    if (context == NULL) {
        return;
    }

    if (context->motion_mode == MOTION_DONE || context->motion_mode == MOTION_FAULT) {
        // MOTION_FAULT: the e-stop stopped this move short of its target.
        const int error = context->motion_mode == MOTION_DONE ? 0 : -ECANCELED;
        context->motion_mode = MOTION_IDLE;
        if (ack_pending[motor_id]) {
            jerrycan_send_ack(context->uuid, error);
            ack_pending[motor_id] = false;
        }
    }
}

static int servo_handler(const jerrycan_msg_t *msg) {
    int rc;

    // If we receive a servo message, we should move the servo
    LOG_INF(
        "Received servo move message: motor_id=%d, abs_or_rel=%d, position=%f, max_velocity=%f, "
        "max_acceleration=%f uuid=%d",
        msg->servo_move.motor_id, msg->servo_move.abs_or_rel, (double)msg->servo_move.position,
        (double)msg->servo_move.max_velocity, (double)msg->servo_move.max_acceleration, (int)msg->uuid);

    const uint8_t motor_id = msg->servo_move.motor_id;
    const struct device *dev = servo_motor_by_id(motor_id);
    struct servo_work_context *context = find_servo_context_from_device(dev);
    if (!context) {
        return -ENOENT;
    }

    // A finished move the status work hasn't acknowledged yet is acknowledged now, so its uuid isn't overwritten.
    servo_consume_completion(motor_id);
    if (ack_pending[motor_id]) {
        // Still running, or it finished after the check above and the next status work acknowledges it.
        LOG_ERR("Servo %d move refused: the previous one hasn't completed", motor_id);
        return -EBUSY;
    }

    // Claim the completion before starting, in case the move finishes first.
    context->uuid = msg->uuid;
    ack_pending[motor_id] = true;

    switch (msg->servo_move.abs_or_rel) {
        case JERRYCAN_MOVE_ABSOLUTE:
            // Move the servo to the absolute position
            rc = servo_move_to_position(dev, msg->servo_move.position, msg->servo_move.max_velocity,
                                        msg->servo_move.max_acceleration);
            break;

        case JERRYCAN_MOVE_RELATIVE:
            // Move the servo to the relative position
            rc = servo_move_relative(dev, msg->servo_move.position, msg->servo_move.max_velocity,
                                     msg->servo_move.max_acceleration);
            break;

        default:
            rc = -EINVAL;
            LOG_ERR("Invalid move type: %d", msg->servo_move.abs_or_rel);
            break;
    }

    if (rc != 0) {
        ack_pending[motor_id] = false;
        return rc;
    }

    return COMMAND_NOT_COMPLETE;
}

static jerrycan_rx_callback_t servo_callback = {
    .filter_msg_type = JERRYCAN_CMD_SERVO_MOVE,
    .func = servo_handler,
};

static int servo_attach_handler(const jerrycan_msg_t *msg) {
    const struct device *dev = servo_motor_by_id(msg->servo_move.motor_id);

    ll_servo_enable(dev, true);

    return SEND_NO_ACKNOWLEDGEMENT;
}

static jerrycan_rx_callback_t servo_attach_callback = {
    .filter_msg_type = JERRYCAN_CMD_SERVO_ATTACH,
    .func = servo_attach_handler,
};

static int servo_detach_handler(const jerrycan_msg_t *msg) {
    const struct device *dev = servo_motor_by_id(msg->servo_move.motor_id);

    ll_servo_enable(dev, false);

    return SEND_NO_ACKNOWLEDGEMENT;
}

static jerrycan_rx_callback_t servo_detach_callback = {
    .filter_msg_type = JERRYCAN_CMD_SERVO_DETACH,
    .func = servo_detach_handler,
};

static int servo_cfg_write_handler(const jerrycan_msg_t *msg) {
    // Ignore cfg writes that are not servo settings
    if (msg->cfg_write.type != JERRYCAN_CFG_SERVO) {
        return SEND_NO_ACKNOWLEDGEMENT;
    }

    LOG_INF(
        "Received servo config write message: motor_id=%d, min_position=%f, max_position=%f, min_pwm_duration_us=%f, "
        "max_pwm_duration_us=%f, motor_max_vel=%f, motor_max_accel=%f",
        msg->cfg_write.servo.motor_id, (double)msg->cfg_write.servo.min_position,
        (double)msg->cfg_write.servo.max_position, (double)msg->cfg_write.servo.min_pwm_duration_us,
        (double)msg->cfg_write.servo.max_pwm_duration_us, (double)msg->cfg_write.servo.motor_max_velocity,
        (double)msg->cfg_write.servo.motor_max_acceleration);

    const struct device *dev = servo_motor_by_id(msg->cfg_write.servo.motor_id);
    int rc;
    if (dev == NULL) {
        LOG_ERR("Invalid servo device number: %d", msg->cfg_write.servo.motor_id);
        rc = -ENODEV;
    } else {
        rc = servo_set_angle_parameters(dev, msg->cfg_write.servo.min_position, msg->cfg_write.servo.max_position);
        int rc2 = servo_set_parameters(
            dev, msg->cfg_write.servo.motor_max_velocity, msg->cfg_write.servo.motor_max_acceleration,
            msg->cfg_write.servo.min_pwm_duration_us, msg->cfg_write.servo.max_pwm_duration_us);
        rc = (rc != 0 ? rc : rc2);
    }

    return rc;
}

static jerrycan_rx_callback_t servo_cfg_write_callback = {
    .filter_msg_type = JERRYCAN_CMD_CFG_WRITE,
    .func = servo_cfg_write_handler,
};

static int servo_cfg_read_handler(const jerrycan_msg_t *msg) {
    // Ignore cfg reads that are not servo settings
    if (msg->cfg_write.type != JERRYCAN_CFG_SERVO) {
        return SEND_NO_ACKNOWLEDGEMENT;
    }

    jerrycan_msg_t rsp;
    rsp.type = JERRYCAN_CMD_CFG_RESPONSE;
    rsp.cfg_response.type = JERRYCAN_CFG_SERVO;
    rsp.cfg_response.servo.motor_id = msg->cfg_write.servo.motor_id;
    rsp.cfg_response.servo.error = false;

    LOG_DBG("Servo Configuration Requested");

    const struct device *dev = servo_motor_by_id(msg->cfg_write.servo.motor_id);
    if (dev == NULL) {
        LOG_ERR("Invalid servo device number: %d", msg->cfg_write.servo.motor_id);
        rsp.cfg_response.servo.error = true;
    } else {
        servo_config_t config;
        int ret = servo_read_config(dev, &config);
        if (ret < 0) {
            LOG_ERR("Failed to get configuration: %d", ret);
            rsp.cfg_response.servo.error = true;
        } else {
            rsp.cfg_response.servo.min_position = config.min_position;
            rsp.cfg_response.servo.max_position = config.max_position;
            rsp.cfg_response.servo.min_pwm_duration_us = config.min_pwm_duration_us;
            rsp.cfg_response.servo.max_pwm_duration_us = config.max_pwm_duration_us;
            rsp.cfg_response.servo.motor_max_velocity = config.motor_max_velocity;
            rsp.cfg_response.servo.motor_max_acceleration = config.motor_max_acceleration;
        }
    }

    jerrycan_tx(&rsp, K_NO_WAIT);

    return SEND_NO_ACKNOWLEDGEMENT;
}

static jerrycan_rx_callback_t servo_cfg_read_callback = {
    .filter_msg_type = JERRYCAN_CMD_CFG_READ,
    .func = servo_cfg_read_handler,
};

static void jerrycan_servo_status_tx(struct k_work *work) {
    ARG_UNUSED(work);

    for (int motor_id = 0; motor_id < SERVO_COUNT; motor_id++) {
        const struct device *servo = servo_motor_by_id(motor_id);

        if (servo == NULL) {
            LOG_WRN("Failed to retreive servo for servo_status_tx: Invalid servo device number: %d", motor_id);
            continue;
        }
        struct servo_work_context *context = find_servo_context_from_device(servo);

        servo_consume_completion(motor_id);

        jerrycan_msg_t msg = {
            .type = JERRYCAN_CMD_SERVO_STATUS,
            .servo_status = {
                .motor_id = motor_id,
                .status = 0,  // ll_motor_get_status(servo),  // FIXME: Needs to be implemented or removed
                .position = context->context.known_position,
            }};

        /* Transmit message */
        int ret = jerrycan_tx(&msg, K_NO_WAIT);
        if (ret != 0) {
            LOG_WRN("Failed to send servo Status CAN message for servo%d: %d", motor_id, ret);
        }
    }
}

static K_WORK_DEFINE(jerrycan_servo_status_tx_work, jerrycan_servo_status_tx);

// The completion sweep shares `ack_pending` and the uuid with the move handler, so it runs in a cooperative thread
// like the handler rather than in the timer ISR, where it could interleave with a command's claim.
static void jerrycan_servo_status_tx_timer_expiry(struct k_timer *timer) {
    ARG_UNUSED(timer);
    k_work_submit(&jerrycan_servo_status_tx_work);
}

K_TIMER_DEFINE(jerrycan_servo_status_tx_timer, jerrycan_servo_status_tx_timer_expiry, NULL);

static int jerrycan_servo_init() {
    jerrycan_register_rx_callback(&servo_callback);
    jerrycan_register_rx_callback(&servo_attach_callback);
    jerrycan_register_rx_callback(&servo_detach_callback);
    jerrycan_register_rx_callback(&servo_cfg_write_callback);
    jerrycan_register_rx_callback(&servo_cfg_read_callback);

    /* Start timer to send the servo status messages periodically */
    k_timer_start(&jerrycan_servo_status_tx_timer, K_MSEC(100), K_MSEC(CONFIG_LIB_JERRYCAN_SERVO_STATUS_TX_PERIOD_MS));

    return 0;
}

SYS_INIT(jerrycan_servo_init, APPLICATION, CONFIG_LIB_JERRYCAN_INIT_PRIORITY);
