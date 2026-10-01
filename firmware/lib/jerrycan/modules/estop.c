/**
 * @file estop.c
 * @brief JerryCAN Emergency Stop Message Handling
 *
 * An ESTOP message with a nonzero payload engages the e-stop: every stepper and servo stops where it is, moves in
 * progress are acknowledged with -ECANCELED, and new moves and homing are refused with -ECANCELED. An ESTOP with a
 * zero payload releases it. A stopped stepper's position is unproven, so steppers must also be homed before they
 * move again.
 *
 * ESTOP frames are taken from the CAN ISR rather than the RX queue (see `jerrycan_set_estop_handler`). The ISR
 * latches the e-stop at once, so nothing new can start; stopping the motors blocks, so a dedicated work queue that
 * outranks both `jerrycan_run()` and the motor work queue does that.
 *
 * The host broadcasts ESTOP with uuid 0, so it isn't acknowledged. Instead, once the e-stop has been applied, each
 * module sends a STATUS message whose `estop_active` reports its latch; a module the host doesn't hear from didn't
 * act on the e-stop. The STATUS message's other fields aren't populated.
 */

#include <zephyr/logging/log.h>

#include "jerrycan.h"
#include "motor_motion.h"

LOG_MODULE_DECLARE(jerrycan, CONFIG_LIB_JERRYCAN_LOG_LEVEL);

static struct k_work_q estop_workq;
static K_THREAD_STACK_DEFINE(estop_workq_stack, CONFIG_LIB_JERRYCAN_ESTOP_STACK_SIZE);
static struct k_work estop_work;

// Set by an engaging frame and taken by the work item, so an engage followed at once by a release still stops the
// motors before the latch is released.
static atomic_t stop_pending;
// Whether the latest frame released the e-stop.
static atomic_t release_requested;

static void estop_send_status(void) {
    jerrycan_msg_t msg = {
        .type = JERRYCAN_CMD_STATUS,
        .status =
            {
                .estop_active = e_stop_engaged(),
            },
        .uuid = 0,
    };

    jerrycan_tx(&msg, K_NO_WAIT);
}

static void estop_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (atomic_cas(&stop_pending, 1, 0)) {
        LOG_WRN("E-stop engaged");
        trigger_e_stop();
    }

    if (atomic_get(&release_requested) && e_stop_engaged()) {
        LOG_INF("E-stop released");
        release_e_stop();
    }

    estop_send_status();
}

static void estop_received(const bool engage) {
    if (engage) {
        latch_e_stop();
        atomic_set(&stop_pending, 1);
    }
    atomic_set(&release_requested, !engage);

    k_work_submit_to_queue(&estop_workq, &estop_work);
}

static int jerrycan_estop_init() {
    k_work_init(&estop_work, estop_work_handler);
    k_work_queue_init(&estop_workq);
    k_work_queue_start(&estop_workq, estop_workq_stack, K_THREAD_STACK_SIZEOF(estop_workq_stack),
                       CONFIG_LIB_JERRYCAN_ESTOP_PRIORITY, NULL);

    jerrycan_set_estop_handler(estop_received);

    return 0;
}

SYS_INIT(jerrycan_estop_init, APPLICATION, CONFIG_LIB_JERRYCAN_INIT_PRIORITY);
