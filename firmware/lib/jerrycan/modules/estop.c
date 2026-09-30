/**
 * @file estop.c
 * @brief JerryCAN Emergency Stop Message Handling
 *
 * An ESTOP message with a nonzero payload engages the e-stop: every stepper and servo stops where it is, moves in
 * progress are acknowledged with -ECANCELED, and new moves and homing are refused with -ECANCELED. An ESTOP with a
 * zero payload releases it. A stopped stepper's position is unproven, so steppers must also be homed before they
 * move again.
 *
 * The host broadcasts ESTOP with uuid 0, so it isn't acknowledged.
 */

#include <zephyr/logging/log.h>

#include "jerrycan.h"
#include "motor_motion.h"

LOG_MODULE_DECLARE(jerrycan, CONFIG_LIB_JERRYCAN_LOG_LEVEL);

static int estop_handler(const jerrycan_msg_t *msg) {
    if (msg->estop.rsvd != 0) {
        LOG_WRN("E-stop engaged");
        trigger_e_stop();
    } else {
        LOG_INF("E-stop released");
        release_e_stop();
    }

    return SEND_NO_ACKNOWLEDGEMENT;
}

static jerrycan_rx_callback_t estop_callback = {
    .filter_msg_type = JERRYCAN_CMD_ESTOP,
    .func = estop_handler,
};

static int jerrycan_estop_init() {
    jerrycan_register_rx_callback(&estop_callback);

    return 0;
}

SYS_INIT(jerrycan_estop_init, APPLICATION, CONFIG_LIB_JERRYCAN_INIT_PRIORITY);
