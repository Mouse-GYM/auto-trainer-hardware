/**
 * @file stepper.c
 * @brief JerryCAN Stepper Motor Message Handling
 *
 * This file handles the reception and processing of stepper motor control messages
 * recieved via the JerryCAN library. It provides functionality to move the stepper,
 * as well as to read and write stepper configuration settings over CAN. The module
 * integrates with the motor library for controlling and configuring stepper movements.
 *
 * Key Functions:
 * - `stepper_handler()`: Processes incoming stepper movement commands and logs the
 *    motion parameters. Integration with the motion library is pending.
 * - `stepper_cfg_write_handler()`: Handles configuration write messages to set
 *    stepper parameters such as minimum and maximum positions.
 * - `stepper_cfg_read_handler()`: Responds to configuration read requests by sending
 *    current stepper configuration parameters back over CAN. Reading actual values
 *    from the motor library is pending.
 * - `jerrycan_stepper_init()`: Registers callbacks to handle CAN messages related
 *    to stepper movement, configuration write, and configuration read operations.
 *
 * Dependencies:
 * - `jerrycan_register_rx_callback()`: Registers a CAN message callback in the JerryCAN system.
 * - `jerrycan_tx()`: Transmits CAN messages via the JerryCAN library.
 *
 * Usage:
 * This module initializes at startup using Zephyr’s SYS_INIT macro. It registers
 * callbacks for handling movement and configuration messages related to stepper motors.
 * Future integration with the high-level motion library will allow actual motor movements
 * and configurations.
 */

#include "stepper.h"

#include <zephyr/logging/log.h>

#include "jerrycan.h"
#include "motor_motion.h"
#include "motor_motion_workq.h"

LOG_MODULE_DECLARE(jerrycan, CONFIG_LIB_JERRYCAN_LOG_LEVEL);

#define DT_DRV_COMPAT ll_stepper

/* Number of enabled steppers found in the device tree */
#define STEPPER_COUNT DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)

typedef enum {
    MOVING_NONE = 0,
    MOVING_SINGLE,
    MOVE_X,
    MONITOR_X,
    MOVE_Y,
    MONITOR_Y,
    MOVE_Z,
    MONITOR_Z,
    MOVING_COMPLETE
} moving_state_t;

static moving_state_t moving_state = MOVING_NONE;

static int stepper_move_handler(const jerrycan_msg_t *msg) {
    // If we receive a stepper message, we should move the stepper
    LOG_INF(
        "Received stepper move message: motor_id=%d, abs_or_rel=%d, position=%f, max_velocity=%f, "
        "max_acceleration=%f",
        msg->stepper_move.motor_id, msg->stepper_move.abs_or_rel, (double)msg->stepper_move.position,
        (double)msg->stepper_move.max_velocity, (double)msg->stepper_move.max_acceleration);

    const jerrycan_cmd_stepper_move_t *move = &msg->stepper_move;

    const struct device *dev = stepper_motor_by_id(move->motor_id);
    struct stepper_work_context *context = find_stepper_context_from_device(dev);
    if (!context) {
        return -ENODEV;
    }

    int rc;

    if (move->save) {
        stepper_save_fixed_location(context, move->motor_id, move->position, move->abs_or_rel == JERRYCAN_MOVE_ABSOLUTE);
        rc = 0;
    } else {
        moving_state = MOVING_SINGLE;

        switch (msg->stepper_move.abs_or_rel) {
            case JERRYCAN_MOVE_ABSOLUTE:
                // Move the stepper to the absolute position
                rc = stepper_move_to_position(dev, move->position, move->max_velocity, move->max_acceleration);
                break;

            case JERRYCAN_MOVE_RELATIVE:
                // Move the stepper to the relative position
                rc = stepper_move_relative(dev, move->position, move->max_velocity, move->max_acceleration);
                break;

            default:
                LOG_ERR("Invalid move type: %d", msg->stepper_move.abs_or_rel);
                rc = -1;
        }

        if (rc == 0) {
            // only retain/associate uuid with context if command was accepted
            context->uuid = msg->uuid;
            rc = COMMAND_NOT_COMPLETE;
        }
    }
    return rc;
}

static jerrycan_rx_callback_t stepper_callback = {
    .filter_msg_type = JERRYCAN_CMD_STEPPER_MOVE,
    .func = stepper_move_handler,
};

static int get_motor_id_for_state(moving_state_t state) {
    switch (state) {
        case MOVE_X:
        case MONITOR_X:
            return 0;
        case MOVE_Y:
        case MONITOR_Y:
            return 1;
        case MOVE_Z:
        case MONITOR_Z:
            return 2;
        default:
            return -1;  // Invalid state
    }
}

static int set_uuid_for_xyz_context(const uint8_t uuid) {
    // check all are free before
    for (int motor_id = 0; motor_id < 3; motor_id++) {
        const struct device *dev = stepper_motor_by_id(motor_id);
        struct stepper_work_context *context = find_stepper_context_from_device(dev);
        if (context->motion_mode == MOTION_IN_PROGESS) {
            return -EBUSY;
        }
    }
    // and only then assign uuid:
    for (int motor_id = 0; motor_id < 3; motor_id++) {
        const struct device *dev = stepper_motor_by_id(motor_id);
        struct stepper_work_context *context = find_stepper_context_from_device(dev);
        context->uuid = uuid;
    }
    return 0;
}

static bool attempt_motor_move(int motor_id) {
    const struct device *dev = stepper_motor_by_id(motor_id);
    struct stepper_work_context *context = find_stepper_context_from_device(dev);

    LOG_WRN("Attempt move for Motor %d", motor_id);
    const int rc = stepper_move_to_position(dev, context->fixed_position, context->motor_max_velocity,
                                            context->motor_max_acceleration);

    if (rc < 0) {
        context->motion_mode = MOTION_DONE;
        return false;
    }

    return true;
}

static bool is_motor_motion_complete(moving_state_t state) {
    const int motor_id = get_motor_id_for_state(moving_state);
    const struct device *dev = stepper_motor_by_id(motor_id);
    const struct stepper_work_context *context = find_stepper_context_from_device(dev);

    return (context->motion_mode == MOTION_DONE);
}

static void stepper_handle_motion_complete() {
    for (int i = 0; i < STEPPER_COUNT; ++i) {
        const struct device *dev = stepper_motor_by_id(i);
        struct stepper_work_context *context = find_stepper_context_from_device(dev);
        if (context && context->motion_mode == MOTION_DONE) {
            LOG_INF("Motion Complete for %d. state=%d. uuid=%d", i, moving_state, context->uuid);
            context->motion_mode = MOTION_IDLE;
            if (moving_state == MOVING_SINGLE || moving_state == MOVING_COMPLETE) {
                jerrycan_send_ack(context->uuid, 0);
                moving_state = MOVING_NONE;
            }
        }
    }
}

static void stepper_handle_fixed_sequence() {
    int motor_id;

    switch (moving_state) {
        case MOVE_X:
            motor_id = get_motor_id_for_state(moving_state);
            moving_state = attempt_motor_move(motor_id) ? MONITOR_X : MOVE_Z;
            break;

        case MONITOR_X:
            if (is_motor_motion_complete(moving_state)) {
                moving_state = MOVE_Z;
            }
            break;

        case MOVE_Y:
            motor_id = get_motor_id_for_state(moving_state);
            moving_state = attempt_motor_move(motor_id) ? MONITOR_Y : MOVING_COMPLETE;
            break;

        case MONITOR_Y:
            if (is_motor_motion_complete(moving_state)) {
                moving_state = MOVING_COMPLETE;
            }
            break;

        case MOVE_Z:
            motor_id = get_motor_id_for_state(moving_state);
            moving_state = attempt_motor_move(motor_id) ? MONITOR_Z : MOVE_Y;
            break;

        case MONITOR_Z:
            if (is_motor_motion_complete(moving_state)) {
                moving_state = MOVE_Y;
            }
            break;

        case MOVING_NONE:
        case MOVING_SINGLE:
            break;

        default:
            moving_state = MOVING_COMPLETE;
            break;
    }

    stepper_handle_motion_complete();
}

static int stepper_fixed_move(const jerrycan_msg_t *msg) {
    int rc = set_uuid_for_xyz_context(msg->uuid);
    if (rc == 0) {
        moving_state = MOVE_X;
    }

    return rc == 0 ? COMMAND_NOT_COMPLETE : rc;
}

static jerrycan_rx_callback_t stepper_fixed_callback = {
    .filter_msg_type = JERRYCAN_CMD_FIXED_XYZ,
    .func = stepper_fixed_move,
};

static int stepper_cfg_write_handler(const jerrycan_msg_t *msg) {
    // Ignore cfg writes that are not stepper settings
    if (msg->cfg_write.type != JERRYCAN_CFG_STEPPER) {
        return SEND_NO_ACKNOWLEDGEMENT;
    }

    LOG_INF(
        "Received stepper config write message: motor_id=%d, microsteps=%d, steps_per_revolution=%f,"
        "max vel=%f, max accel=%f, flip limit=%d",
        msg->cfg_write.stepper.motor_id, msg->cfg_write.stepper.microsteps,
        (double)msg->cfg_write.stepper.steps_per_revolution, (double)msg->cfg_write.stepper.motor_max_velocity,
        (double)msg->cfg_write.stepper.motor_max_acceleration, msg->cfg_write.stepper.flip_limit_orientation);

    int rc;
    const struct device *dev = stepper_motor_by_id(msg->cfg_write.stepper.motor_id);
    if (dev == NULL) {
        LOG_ERR("Invalid stepper device number: %d", msg->cfg_write.stepper.motor_id);
        rc = -ENODEV;
    } else {
        rc = stepper_set_parameters(
            dev, msg->cfg_write.stepper.motor_max_velocity, msg->cfg_write.stepper.motor_max_acceleration,
            msg->cfg_write.stepper.homing_velocity, msg->cfg_write.stepper.microsteps,
            msg->cfg_write.stepper.steps_per_revolution, msg->cfg_write.stepper.flip_limit_orientation);
    }

    return rc;
}

static jerrycan_rx_callback_t stepper_cfg_write_callback = {
    .filter_msg_type = JERRYCAN_CMD_CFG_WRITE,
    .func = stepper_cfg_write_handler,
};

static int stepper_cfg_read_handler(const jerrycan_msg_t *msg) {
    // Ignore cfg reads that are not stepper settings
    if (msg->cfg_write.type != JERRYCAN_CFG_STEPPER) {
        return SEND_NO_ACKNOWLEDGEMENT;
    }

    jerrycan_msg_t rsp;
    rsp.type = JERRYCAN_CMD_CFG_RESPONSE;
    rsp.cfg_response.type = JERRYCAN_CFG_STEPPER;
    rsp.cfg_response.stepper.motor_id = msg->cfg_write.stepper.motor_id;
    rsp.cfg_response.stepper.error = false;

    const struct device *motor = stepper_motor_by_id(msg->cfg_write.stepper.motor_id);

    if (motor == NULL) {
        LOG_ERR("Invalid motor id: %d", msg->cfg_write.stepper.motor_id);
        rsp.cfg_response.stepper.error = true;
    } else {
        stepper_config_t cfg;
        const int ret = stepper_read_config(motor, &cfg);
        if (ret < 0) {
            LOG_ERR("Failed to read motor config: %d", ret);
            rsp.cfg_response.stepper.error = true;
        } else {
            rsp.cfg_response.stepper.flip_limit_orientation = cfg.flip_limit_orientation;
            rsp.cfg_response.stepper.steps_per_revolution = cfg.steps_per_revolution;
            rsp.cfg_response.stepper.motor_max_velocity = cfg.motor_max_velocity;
            rsp.cfg_response.stepper.homing_velocity = cfg.homing_velocity;
            rsp.cfg_response.stepper.motor_max_acceleration = cfg.motor_max_acceleration;
            rsp.cfg_response.stepper.microsteps = cfg.microsteps;
        }
    }

    jerrycan_tx(&rsp, K_NO_WAIT);

    return SEND_NO_ACKNOWLEDGEMENT;
}

static jerrycan_rx_callback_t stepper_cfg_read_callback = {
    .filter_msg_type = JERRYCAN_CMD_CFG_READ,
    .func = stepper_cfg_read_handler,
};

static int stepper_home_handler(const jerrycan_msg_t *msg) {
    int rc;

    const struct device *dev = stepper_motor_by_id(msg->stepper_home.motor_id);
    struct stepper_work_context *context = find_stepper_context_from_device(dev);
    if (!context) {
        LOG_ERR("Failed to home stepper motor: Invalid stepper device number - %d", msg->stepper_home.motor_id);
        rc = -ENODEV;
    } else {
        LOG_INF("Homing motor %d with UUID=%d", msg->stepper_home.motor_id, msg->uuid);
        moving_state = MOVING_SINGLE;

        rc = stepper_home(dev);
        if (rc == 0) {
            context->uuid = msg->uuid;
        }
    }

    return rc == 0 ? COMMAND_NOT_COMPLETE : rc;
}

static jerrycan_rx_callback_t stepper_home_callback = {
    .filter_msg_type = JERRYCAN_CMD_STEPPER_HOME,
    .func = stepper_home_handler,
};

static void jerrycan_stepper_status_tx(const struct device *stepper) {
    const struct stepper_work_context *context = find_stepper_context_from_device(stepper);

    const uint8_t motor_id = ll_motor_get_id(stepper);
    const float position = context->context.last_position_generated;
    const float send_position = context->fixed_position;

    jerrycan_msg_t msg = {.type = JERRYCAN_CMD_STEPPER_STATUS,
                          .stepper_status = {
                              .motor_id = motor_id,
                              .status = 0,  // TODO: Do something with this?
                              .homing_status = stepper_homing_status(stepper),
                              .position = position,
                              .send_position = send_position,
                              .limit_switch = ll_stepper_get_limit_switch_state(stepper),
                          }};

    /* Transmit message */
    const int ret = jerrycan_tx(&msg, K_NO_WAIT);
    if (ret != 0) {
        LOG_WRN("Failed to send Stepper Status CAN message for stepper%d: %d", motor_id, ret);
    }
}

static void jerrycan_bulk_stepper_status_tx() {
    stepper_handle_fixed_sequence();

    for (int motor_id = 0; motor_id < STEPPER_COUNT; motor_id++) {
        const struct device *stepper = stepper_motor_by_id(motor_id);

        if (stepper) {
            jerrycan_stepper_status_tx(stepper);
        }
    }
}

K_TIMER_DEFINE(jerrycan_stepper_status_tx_timer, jerrycan_bulk_stepper_status_tx, NULL);

static int jerrycan_stepper_init() {
    jerrycan_register_rx_callback(&stepper_callback);
    jerrycan_register_rx_callback(&stepper_cfg_write_callback);
    jerrycan_register_rx_callback(&stepper_cfg_read_callback);
    jerrycan_register_rx_callback(&stepper_home_callback);
    jerrycan_register_rx_callback(&stepper_fixed_callback);

    /* Start timer to send the stepper status messages periodically */
    k_timer_start(&jerrycan_stepper_status_tx_timer, K_MSEC(100),
                  K_MSEC(CONFIG_LIB_JERRYCAN_STEPPER_STATUS_TX_PERIOD_MS));

    return 0;
}

SYS_INIT(jerrycan_stepper_init, APPLICATION, CONFIG_LIB_JERRYCAN_INIT_PRIORITY);
