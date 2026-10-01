#include "motor_motion_workq.h"

#include <math.h>
#include <stdatomic.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "adi_tmc2209.h"
#include "jerrycan.h"
#include "motor_common.h"
#include "motor_motion.h"
#include "motor_settings.h"
#include "servo.h"
#include "stepper.h"

LOG_MODULE_DECLARE(motor_motion, CONFIG_LIB_MOTOR_MOTION_LOG_LEVEL);

/* ***** Forward Declaration of Callbacks ***** */
#ifdef CONFIG_DT_HAS_LL_SERVO_ENABLED
static void servo_motor_event_callback(const struct device *const dev, ll_motor_events_t event, void *arg,
                                       void *user_data);
#endif

#ifdef CONFIG_DT_HAS_LL_STEPPER_ENABLED
static void stepper_motor_event_callback(const struct device *const dev, ll_motor_events_t event, void *arg,
                                         void *user_data);
#endif

static void stepper_work_homing_verify_handler(struct k_work *work);
static int stepper_start_move(struct stepper_work_context *context, float target_position, float max_velocity,
                              float max_acceleration);

/* ***** Static Context Structs Used Throughout ***** */

// Default pwm duration of the minimum angle
#define SERVO_DEFAULT_MIN_ANGLE_PWM 1000.0f
// Default pwm duration of the maximum angle
#define SERVO_DEFAULT_MAX_ANGLE_PWM 2000.0f
// Default minimum angle of servo
#define SERVO_DEFAULT_MIN_ANGLE 0.0f
// Default maximum angle of servo
#define SERVO_DEFAULT_MAX_ANGLE 180.0f
// Default angular adjustment of servo
#define SERVO_DEFAULT_ANGLE_ADJUSTMENT SERVO_DEFAULT_MIN_ANGLE
// Default 'max_velocity' of servo
#define SERVO_DEFAULT_MAX_VELOCITY 200
// Default 'max_acceleration' of servo
#define SERVO_DEFAULT_MAX_ACCELERATION 100

// Period between successive status checks (DRV_STATUS and move timeout) of a moving stepper
#define STEPPER_DRIVER_CHECK_PERIOD 100U
// Consecutive failed DRV_STATUS reads during a move before the driver counts as unreachable
#define STEPPER_DRV_STATUS_READ_FAILURES 3U
// Pause before each step of the homing verification, so the carriage and the switch contact settle after a stop
#define STEPPER_HOMING_VERIFY_SETTLE_MS 20U
// Default 'min_step' of stepper (number of steps, incl. microstepping, done per pulse)
#define STEPPER_DEFAULT_STEPS_PER_REVOLUTION 48.0f
// Default 'max_velocity' of stepper
#define STEPPER_DEFAULT_MAX_VELOCITY 20.0f
// Default 'max_acceleration' of stepper
#define STEPPER_DEFAULT_MAX_ACCELERATION 100.0f
// Default 'flip_limit_orientation' of stepper
#define STEPPER_DEFAULT_FLIP_LIMIT_ORIENTATION false
// Default 'micro_steps' of stepper
#define STEPPER_DEFAULT_MICRO_STEPS 1
// Fewest step-timer ticks between pulses a move or homing may ask for. The timing generator replaces anything at or
// below one tick with a fixed fast interval, flattening the ramp, so velocities are kept well clear of that.
#define STEPPER_MIN_PULSE_TICKS 4.0f

#define DEV_DEFINE_SERVO_CONTEXT(id)                                                         \
    {.dev = DEVICE_DT_GET(id),                                                               \
     .context =                                                                              \
         {                                                                                   \
             .motion_profile =                                                               \
                 {                                                                           \
                     .start_pos = 0,                                                         \
                     .end_pos = 0,                                                           \
                     .a_max = 0,                                                             \
                     .v_max = 0,                                                             \
                     .sgn = 0,                                                               \
                     .y_f = 0,                                                               \
                     .y_s = 0,                                                               \
                     .y_a = 0,                                                               \
                     .v_w = 0,                                                               \
                     .t_o = 0,                                                               \
                     .t_a = 0,                                                               \
                     .omega = 0,                                                             \
                     .k_s = 0,                                                               \
                     .t_k = 0,                                                               \
                     .t_s = 0,                                                               \
                     .t_t = 0,                                                               \
                 },                                                                          \
             .min_angle = SERVO_DEFAULT_MIN_ANGLE,                                           \
             .max_angle = SERVO_DEFAULT_MAX_ANGLE,                                           \
             .min_angle_pwm = SERVO_DEFAULT_MIN_ANGLE_PWM,                                   \
             .max_angle_pwm = SERVO_DEFAULT_MAX_ANGLE_PWM,                                   \
             .pwm_timer_increment = (DT_PROP(DT_PARENT(id), st_prescaler) + 1.0f) / 170.0f,  \
             .last_time_generated = 0.0f,                                                    \
             .last_position_generated = SERVO_DEFAULT_MIN_ANGLE,                             \
             .known_position = SERVO_DEFAULT_MIN_ANGLE,                                      \
         },                                                                                  \
     .buffers = {{0}},                                                                       \
     .current_buffer = 0,                                                                    \
     .last_calculation_ret = 0,                                                              \
     .e_stop_triggered = {.__val = 0},                                                       \
     .motion_mode = MOTION_IDLE,                                                             \
     .motion_calculation_done = true,                                                        \
     .position_assumed = true,                                                               \
     .pending_position = NAN,                                                                \
     .persisted_position = NAN,                                                              \
     .calculation_work =                                                                     \
         {                                                                                   \
             .work =                                                                         \
                 {                                                                           \
                     .node = {.next = NULL},                                                 \
                     .handler = NULL,                                                        \
                     .queue = NULL,                                                          \
                     .flags = 0,                                                             \
                 },                                                                          \
             .timeout = {.node = {{.head = NULL}, {.tail = NULL}}, .fn = NULL, .dticks = 0}, \
             .queue = NULL,                                                                  \
         },                                                                                  \
     .servo_cb = {                                                                           \
         .func = servo_motor_event_callback,                                                 \
         .user_data = NULL,                                                                  \
         .node = {.next = NULL},                                                             \
     }},

#define DEV_DEFINE_STEPPER_CONTEXT(id)                                                          \
    {                                                                                           \
        .dev = DEVICE_DT_GET(id),                                                               \
        .context =                                                                              \
            {                                                                                   \
                .motion_profile =                                                               \
                    {                                                                           \
                        .start_pos = 0,                                                         \
                        .end_pos = 0,                                                           \
                        .a_max = 0,                                                             \
                        .v_max = 0,                                                             \
                        .sgn = 0,                                                               \
                        .y_f = 0,                                                               \
                        .y_s = 0,                                                               \
                        .y_a = 0,                                                               \
                        .v_w = 0,                                                               \
                        .t_o = 0,                                                               \
                        .t_a = 0,                                                               \
                        .omega = 0,                                                             \
                        .k_s = 0,                                                               \
                        .t_k = 0,                                                               \
                        .t_s = 0,                                                               \
                        .t_t = 0,                                                               \
                    },                                                                          \
                .min_step = 1.0f / STEPPER_DEFAULT_MICRO_STEPS,                                 \
                .timer_increment = 0.0f,                                                        \
                .steps_per_revolution = 0.0f,                                                   \
                .last_time_generated = 0.0f,                                                    \
                .last_position_generated = 0.0f,                                                \
            },                                                                                  \
        .buffers = {{0}},                                                                       \
        .current_buffer = 0,                                                                    \
        .last_calculation_ret = 0,                                                              \
        .e_stop_triggered = {.__val = 0},                                                       \
        .move_control = MOVING_POSITION,                                                        \
        .motion_mode = MOTION_IDLE,                                                             \
        .motion_calculation_done = true,                                                        \
        .calculation_work =                                                                     \
            {                                                                                   \
                .work =                                                                         \
                    {                                                                           \
                        .node = {.next = NULL},                                                 \
                        .handler = NULL,                                                        \
                        .queue = NULL,                                                          \
                        .flags = 0,                                                             \
                    },                                                                          \
                .timeout = {.node = {{.head = NULL}, {.tail = NULL}}, .fn = NULL, .dticks = 0}, \
                .queue = NULL,                                                                  \
            },                                                                                  \
        .check_driver_work =                                                                    \
            {                                                                                   \
                .work =                                                                         \
                    {                                                                           \
                        .node = {.next = NULL},                                                 \
                        .handler = NULL,                                                        \
                        .queue = NULL,                                                          \
                        .flags = 0,                                                             \
                    },                                                                          \
                .timeout = {.node = {{.head = NULL}, {.tail = NULL}}, .fn = NULL, .dticks = 0}, \
                .queue = NULL,                                                                  \
            },                                                                                  \
        .stepper_cb =                                                                           \
            {                                                                                   \
                .func = stepper_motor_event_callback,                                           \
                .user_data = NULL,                                                              \
                .node = {.next = NULL},                                                         \
            },                                                                                  \
        .motor_max_velocity = STEPPER_DEFAULT_MAX_VELOCITY,                                     \
        .motor_max_acceleration = STEPPER_DEFAULT_MAX_ACCELERATION,                             \
        .homing_velocity = STEPPER_DEFAULT_MAX_VELOCITY,                                        \
        .motor_steps_per_revolution = STEPPER_DEFAULT_STEPS_PER_REVOLUTION,                     \
        .microsteps = STEPPER_DEFAULT_MICRO_STEPS,                                              \
        .flip_limit_orientation = STEPPER_DEFAULT_FLIP_LIMIT_ORIENTATION,                       \
        .timer_increment = (DT_PROP(DT_PARENT(id), st_prescaler) + 1.0f) / 170e6f,              \
        .uuid = 0,                                                                              \
    },

struct stepper_work_context stepper_contexts[] = {DT_FOREACH_STATUS_OKAY(ll_stepper, DEV_DEFINE_STEPPER_CONTEXT)};
struct servo_work_context servo_contexts[] = {DT_FOREACH_STATUS_OKAY(ll_servo, DEV_DEFINE_SERVO_CONTEXT)};

static struct k_work_q motor_workq;
static K_THREAD_STACK_DEFINE(motor_workq_stack, CONFIG_LIB_MOTOR_MOTION_WORK_QUEUE_STACK_SIZE);

// Stepper protection runs apart from `motor_workq`: each DRV_STATUS read blocks for about 10 ms, which must not
// delay the step-buffer refills.
static struct k_work_q stepper_monitor_workq;
#if DT_HAS_COMPAT_STATUS_OKAY(ll_stepper)
static K_THREAD_STACK_DEFINE(stepper_monitor_workq_stack, CONFIG_LIB_MOTOR_MOTION_STEPPER_MONITOR_STACK_SIZE);
#endif

// Held across tripping a fault and across clearing one, so a clear can't re-enable a driver the monitor is still
// disabling.
static K_MUTEX_DEFINE(stepper_fault_lock);

/* ***** Helper Functions ***** */

struct servo_work_context *find_servo_context_from_device(const struct device *dev) {
    if (dev) {
        for (size_t i = 0; i < ARRAY_SIZE(servo_contexts); i++) {
            if (servo_contexts[i].dev == dev) {
                return &servo_contexts[i];
            }
        }
    } else {
        LOG_WRN("NULL Device passed to get servo context");
    }

    return NULL;
}

struct stepper_work_context *find_stepper_context_from_device(const struct device *dev) {
    if (dev) {
        for (size_t i = 0; i < ARRAY_SIZE(stepper_contexts); i++) {
            if (stepper_contexts[i].dev == dev) {
                return &stepper_contexts[i];
            }
        }
    } else {
        LOG_WRN("NULL Device passed to get servo context");
    }

    return NULL;
}

/**
 * The highest velocity the step timer can produce at `context`'s microstep and steps-per-revolution settings,
 * leaving STEPPER_MIN_PULSE_TICKS between pulses.
 */
static float stepper_max_timer_velocity(const struct stepper_work_context *context) {
    return 1.0f / (STEPPER_MIN_PULSE_TICKS * context->timer_increment * context->motor_steps_per_revolution *
                   (float)context->microsteps);
}

void stepper_set_position_to_zero(const struct device *dev) {
    struct stepper_work_context *context = find_stepper_context_from_device(dev);
    if (context == NULL) {
        LOG_ERR("Stepper context not found for device");
        return;
    }

    motor_motion_stepper_set_current_position(&context->context, 0.0f);
    context->motion_mode = MOTION_DONE;
}

/**
 * Stop at the limit switch, take it as position 0, and have the homing verification take its next step. Safe to
 * call from ISRs, and again for a bouncing contact.
 */
static void stepper_homing_at_switch(struct stepper_work_context *context) {
    context->motion_calculation_done = true;
    ll_stepper_abort(context->dev);
    motor_motion_stepper_set_current_position(&context->context, 0.0f);
    k_work_schedule_for_queue(&motor_workq, &context->homing_verify_work, K_MSEC(STEPPER_HOMING_VERIFY_SETTLE_MS));
}

/**
 * End homing with `error` (0 for success) unless something else, a fault or a stop, ended it first. A failure
 * leaves the position unproven, so moves are refused until homing succeeds.
 */
static void stepper_homing_finish(struct stepper_work_context *context, const int error) {
    const unsigned int key = irq_lock();
    if (context->motion_mode == MOTION_IN_PROGESS) {
        context->homing_error = error;
        context->motion_mode = error == 0 ? MOTION_DONE : MOTION_FAULT;
        if (error != 0) {
            atomic_flag_test_and_set(&context->e_stop_triggered);
        }
    }
    context->homing_verify = HOMING_VERIFY_NONE;
    // Report homing as the last command, as it was before the verification's moves.
    context->move_control = MOVING_HOME;
    irq_unlock(key);
}

void servo_set_position_to_zero(const struct device *dev) {
    struct servo_work_context *context = find_servo_context_from_device(dev);
    if (context == NULL) {
        LOG_ERR("Servo context not found for device");
        return;
    }

    motor_motion_servo_set_current_position(&context->context, 0.0f);
    context->motion_calculation_done = true;
}

bool servo_angle_limits_usable(const struct servo_work_context *const context) {
    return context != NULL && isfinite(context->context.min_angle) && isfinite(context->context.max_angle) &&
           context->context.min_angle <= context->context.max_angle;
}

void servo_assume_min_angle_position(struct servo_work_context *const context) {
    if (context == NULL || !context->position_assumed) {
        return;
    }

    // `servo_set_angle_parameters` validates nothing, so a malformed cfg frame can leave a non-finite limit
    // here; a NAN must never reach the belief fields, where the profile maths would propagate it.
    const float position = isfinite(context->context.min_angle) ? context->context.min_angle : SERVO_DEFAULT_MIN_ANGLE;

    context->context.known_position = position;
    context->context.last_position_generated = position;
}

/* ***** Callbacks ***** */
#ifdef CONFIG_DT_HAS_LL_STEPPER_ENABLED
static void stepper_motor_event_callback(const struct device *const dev, ll_motor_events_t event, void *arg,
                                         void *user_data) {
    // Due to the limitations of the GPIO callback mechanism, this data is only available
    // in DMA queue events, not the limit switch event.
    struct stepper_work_context *context = user_data;

    switch (event) {
        case LL_MOTOR_EVENT_DMA_BLOCK_COMPLETE:
            // When a block has been completed, the buffer is free to use for the next calculation, if needed
            LOG_DBG("DMA block complete");
            if (context != NULL && !context->motion_calculation_done) {
                k_work_schedule_for_queue(&motor_workq, &context->calculation_work, K_NO_WAIT);
            }
            break;
        case LL_MOTOR_EVENT_DMA_QUEUE_EMPTY:
            LOG_DBG("MOTION_DONE");
            if (context != NULL) {
                switch (context->move_control) {
                    default:
                    case MOVING_POSITION:
                        // When the driver runs out of data to send, the motion is done
                        if (context->motion_mode == MOTION_IN_PROGESS) {
                            if (context->homing_verify != HOMING_VERIFY_NONE) {
                                // A homing verification move ended; homing isn't done until the next step.
                                k_work_schedule_for_queue(&motor_workq, &context->homing_verify_work,
                                                          K_MSEC(STEPPER_HOMING_VERIFY_SETTLE_MS));
                            } else {
                                context->motion_mode = MOTION_DONE;
                            }
                        }
                        break;

                    case MOVING_HOME:
                        if (!context->motion_calculation_done) {
                            k_work_schedule_for_queue(&motor_workq, &context->calculation_work, K_NO_WAIT);
                        } else if (context->motion_mode == MOTION_IN_PROGESS &&
                                   context->homing_verify == HOMING_VERIFY_NONE) {
                            // Every pulse of the maximum travel went out and the limit switch never stopped
                            // the motor. Disabling the driver needs the UART, so the monitor raises the fault.
                            context->homing_travel_exhausted = true;
                            k_work_reschedule_for_queue(&stepper_monitor_workq, &context->check_driver_work, K_NO_WAIT);
                        }
                        break;
                }
            }
            break;
        case LL_MOTOR_EVENT_LIMIT_SWITCH:
            // Context is NULL during this event because of limitations of the GPIO driver
            context = find_stepper_context_from_device(dev);
            if (context && context->motion_mode == MOTION_IN_PROGESS) {
                if (context->homing_verify == HOMING_VERIFY_RETURN) {
                    // Back onto the switch: where it closes is home, as when homing first found it.
                    context->homing_verify = HOMING_VERIFY_CLOSED;
                    stepper_homing_at_switch(context);
                    break;
                }
                if (context->homing_verify == HOMING_VERIFY_CLOSED) {
                    // Contact bounce; home was taken at the first edge.
                    break;
                }

                switch (context->move_control) {
                    case MOVING_HOME:
                        // Only the first edge counts; the verification that follows ignores contact bounce.
                        if (context->homing_verify == HOMING_VERIFY_NONE) {
                            LOG_WRN("Found Limit Switch. Stopping Motor.");
                            context->homing_verify = HOMING_VERIFY_START;
                            stepper_homing_at_switch(context);
                        }
                        break;

                    default:
                    case MOVING_POSITION:
                        if ((context->motor_direction == LL_STEPPER_DIR_BACKWARD && !context->flip_limit_orientation) ||
                            (context->motor_direction == LL_STEPPER_DIR_FORWARD && context->flip_limit_orientation)) {
                            LOG_WRN("Found Limit Switch. Stopping Motor.");
                            stepper_motor_stop(dev);
                            // Homing puts 0 where the switch closes, so that is where the carriage is now, however
                            // far the move had planned (and `last_position_generated` had run) ahead of it.
                            motor_motion_stepper_set_current_position(&context->context, 0.0f);
                        }
                        break;
                }
            }
            break;

        default:
            LOG_WRN("Unknown stepper motor event: %d", event);
            break;
    }
}
#endif

#ifdef CONFIG_DT_HAS_LL_SERVO_ENABLED
static void servo_motor_event_callback(const struct device *const dev, ll_motor_events_t event, void *arg,
                                       void *user_data) {
    struct servo_work_context *context = user_data;

    switch (event) {
        case LL_MOTOR_EVENT_DMA_BLOCK_COMPLETE:
            // When a block has been completed, the buffer is free to use for the next calculation, if needed
            LOG_WRN("DMA block complete");
            if (context != NULL) {
                context->context.known_position = context->context.last_position_generated;
                if (!context->motion_calculation_done) {
                    k_work_schedule_for_queue(&motor_workq, &context->calculation_work, K_NO_WAIT);
                }
            }
            break;
        case LL_MOTOR_EVENT_DMA_QUEUE_EMPTY: {
            LOG_WRN("MOTION DONE");
            if (context != NULL) {
                context->motion_mode = MOTION_DONE;
                context->context.known_position = context->context.last_position_generated;
                context->position_valid = true;

                // The only place a position becomes persistable: the move ran to completion, so the horn is
                // where the generator left it.
                if (context->persisted_position != context->context.known_position) {
                    context->persisted_position = context->context.known_position;
                    motor_settings_save();
                }
            }
            break;
        }
        case LL_MOTOR_EVENT_LIMIT_SWITCH:
            servo_motor_stop(dev);
            servo_cancel_all_work(dev);
            servo_set_position_to_zero(dev);
            break;
        default:
            LOG_WRN("Unknown servo motor event: %d", event);
            break;
    }
}
#endif

/* ***** Work Handlers ***** */

void stepper_cancel_all_work(const struct device *dev) {
    struct stepper_work_context *context = find_stepper_context_from_device(dev);
    k_work_cancel_delayable(&context->calculation_work);
    k_work_cancel_delayable(&context->check_driver_work);
    k_work_cancel_delayable(&context->homing_verify_work);
    context->homing_verify = HOMING_VERIFY_NONE;

    if (context->motion_mode == MOTION_IN_PROGESS) {
        context->motion_mode = MOTION_DONE;
    }
}

void servo_cancel_all_work(const struct device *dev) {
    struct servo_work_context *context = find_servo_context_from_device(dev);
    k_work_cancel_delayable(&context->calculation_work);
    context->motion_mode = MOTION_DONE;
    context->motion_calculation_done = true;
    context->context.known_position = context->context.last_position_generated;
    context->position_valid = false;
}

static void servo_work_calculation_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct servo_work_context *context = CONTAINER_OF(dwork, struct servo_work_context, calculation_work);

    // The servo was stopped after this refill was scheduled; queuing a block now would restart the DMA.
    if (context->motion_calculation_done) {
        return;
    }

    const ssize_t ret = motor_motion_servo_generate_displacement_table(context->buffers[context->current_buffer],
                                                                       SERVO_BUFFER_SIZE, &context->context);
    context->last_calculation_ret = ret;
    if (ret < 0) {
        LOG_ERR("Error generating servo table: %d", ret);
        return;
    }

    // Entire buffer wasn't needed which indicates all the steps have been planned and no more calculations are required
    // If nothing was calculated, then the motor is already at the target position
    if (ret < SERVO_BUFFER_SIZE) {
        context->motion_calculation_done = true;
    }

    // Add the buffer onto the servo driver queue
    ll_queue_servo_positions(context->dev, context->buffers[context->current_buffer],
                             context->last_calculation_ret * sizeof(uint32_t), K_FOREVER);

    // Increment the buffer pointer
    context->current_buffer = (context->current_buffer + 1) % BUFS_PER_MOTOR;
}

size_t stepper_generate_table_for_homing(const struct stepper_work_context *context, uint32_t *buf) {
    const float slow_pulses_ms = 100.0f;

    const float homing_velocity = MIN(context->homing_velocity, stepper_max_timer_velocity(context));
    const float seconds_per_pulse =
        1 / homing_velocity / context->motor_steps_per_revolution * context->context.min_step;

    // At least one pulse: an empty block would never raise the event that queues the next one.
    size_t n_pulses = (size_t)floorf(slow_pulses_ms / 1000.0f / seconds_per_pulse);
    n_pulses = CLAMP(n_pulses, 1, STEPPER_BUFFER_SIZE);

    // The timer's reload register is 16 bits: a longer interval would be truncated into a much shorter one. The
    // velocity limit above already meets the lower bound; it's repeated so no entry can be 0, which stops the timer.
    const long ticks = CLAMP(lroundf(seconds_per_pulse / context->timer_increment), (long)STEPPER_MIN_PULSE_TICKS,
                             (long)UINT16_MAX);
    for (size_t i = 0; i < n_pulses; i++) {
        buf[i] = (uint32_t)ticks;
    }

    return n_pulses;
}

/**
 * Fill `buf` with the next block of homing pulses: the acceleration ramp of the profile `stepper_home` set up,
 * then constant pulses at the homing velocity until the limit switch stops the motor. There's no ramp down.
 * Blocks stop at `homing_pulses_left`; the block that uses the last of them ends the calculation.
 */
static size_t stepper_generate_homing_block(struct stepper_work_context *context, uint32_t *buf) {
    size_t n_pulses = 0;
    if (!context->homing_ramp_done) {
        const ssize_t ret = motor_motion_stepper_generate_ramp_table(buf, STEPPER_BUFFER_SIZE, &context->context);
        if (ret < STEPPER_BUFFER_SIZE) {
            context->homing_ramp_done = true;
        }
        if (ret > 0) {
            n_pulses = (size_t)ret;
        }
    }

    if (n_pulses == 0) {
        n_pulses = stepper_generate_table_for_homing(context, buf);
    }

    n_pulses = MIN(n_pulses, context->homing_pulses_left);
    context->homing_pulses_left -= n_pulses;
    if (context->homing_pulses_left == 0) {
        context->motion_calculation_done = true;
    }

    return n_pulses;
}

static void stepper_work_calculation_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct stepper_work_context *context = CONTAINER_OF(dwork, struct stepper_work_context, calculation_work);

    // The motor was stopped after this refill was scheduled; queuing a block now would restart the DMA.
    if (context->motion_calculation_done) {
        return;
    }

    switch (context->move_control) {
        default:
        case MOVING_POSITION: {
            const ssize_t ret = motor_motion_stepper_generate_timing_table(context->buffers[context->current_buffer],
                                                                           STEPPER_BUFFER_SIZE, &context->context);
            context->last_calculation_ret = ret;
            if (ret < 0) {
                context->motion_calculation_done = true;
                LOG_ERR("Error generating stepper table.");
                return;
            }

            // Entire buffer wasn't needed which indicates all the steps have been planned and no more calculations are
            // required If nothing was calculated, then the motor is already at the target position
            if (ret < STEPPER_BUFFER_SIZE) {
                context->motion_calculation_done = true;
            }

            // Add the buffer onto the stepper driver queue
            LOG_DBG("Q buf %d [%p]", context->current_buffer, (void *)context->buffers[context->current_buffer]);
            if (ret > 0) {
                // Sending a buffer of length 0 here causes the event callbacks
                // to function a little weirdly.
                ll_queue_stepper_positions(context->dev, context->buffers[context->current_buffer],
                                           context->last_calculation_ret * sizeof(uint32_t), K_FOREVER);
            }

            // Increment the buffer pointer
            context->current_buffer = (context->current_buffer + 1) % BUFS_PER_MOTOR;
        } break;

        case MOVING_HOME: {
            const size_t ret = stepper_generate_homing_block(context, context->buffers[context->current_buffer]);
            context->last_calculation_ret = ret;

            LOG_DBG("Q buf %d [%p]", context->current_buffer, (void *)context->buffers[context->current_buffer]);
            ll_queue_stepper_positions(context->dev, context->buffers[context->current_buffer],
                                       context->last_calculation_ret * sizeof(uint32_t), K_FOREVER);

            context->current_buffer = (context->current_buffer + 1) % BUFS_PER_MOTOR;
        } break;
    }
}

/* ***** Stepper Protection ***** */

static const struct device *stepper_driver_of(const struct stepper_work_context *context) {
    const ll_motor_cfg_t *cfg = context->dev->config;
    return cfg->stepper_driver_device;
}

/**
 * Whether the driver's configuration is confirmed well enough to move: its boot-time check passed, and it took
 * `microsteps`. A failed boot check holds until reset. Replaying the driver's init at runtime would not be a safe
 * way to clear it: init sets MRES to 1 ahead of the motion layer's microstep write, and writes CHOPCONF with a
 * nonzero toff, which would re-enable a driver the protection has switched off.
 */
static bool stepper_driver_config_verified(const struct stepper_work_context *context) {
    const struct device *driver = stepper_driver_of(context);
    if (driver == NULL) {
        return true;
    }

    return adi_tmc2209_get_init_check(driver)->faults == 0 && !context->microsteps_unverified;
}

bool stepper_microsteps_unverified(const struct device *dev) {
    const struct stepper_work_context *context = find_stepper_context_from_device(dev);
    return context != NULL && context->microsteps_unverified;
}

/**
 * Timeout for a move of `distance` revolutions. The nominal duration d/v + 2v/a is the profile's total time t_t
 * when it reaches `velocity`, and bounds t_t = 2 * sqrt(2d/a) from above when it doesn't, so the timeout grows in
 * proportion to the distance.
 */
static int64_t stepper_move_timeout_ms(const float distance, const float velocity, const float acceleration) {
    const float nominal_s = distance / velocity + 2.0f * velocity / acceleration;
    return (int64_t)(nominal_s * (float)CONFIG_LIB_MOTOR_MOTION_STEPPER_MOVE_TIMEOUT_PERCENT * 10.0f) +
           CONFIG_LIB_MOTOR_MOTION_STEPPER_MOVE_TIMEOUT_MARGIN_MS;
}

/**
 * Start polling a move that has just started. Safe to call from ISRs. `move_deadline_ms` must already be set,
 * before `motion_mode` became MOTION_IN_PROGESS.
 */
static void stepper_monitor_start(struct stepper_work_context *context) {
    k_work_reschedule_for_queue(&stepper_monitor_workq, &context->check_driver_work,
                                K_MSEC(STEPPER_DRIVER_CHECK_PERIOD));
}

/**
 * Read the driver's DRV_STATUS and return the STEPPER_FAULT_* bits it warrants. The bus is shared and a reply can
 * be lost, so one failed read passes; STEPPER_DRV_STATUS_READ_FAILURES in a row is a fault, since the
 * temperature can no longer be watched.
 */
static uint32_t stepper_poll_driver(struct stepper_work_context *context) {
    const struct device *driver = stepper_driver_of(context);
    if (driver == NULL) {
        return 0;
    }

    adi_tmc2209_reg_t reg = {0};
    const int ret = adi_tmc2209_read_drv_status(driver, &reg);
    if (ret < 0) {
        context->drv_status_read_failures++;
        LOG_WRN("Stepper %d DRV_STATUS read failed (%d), %u in a row", ll_motor_get_id(context->dev), ret,
                context->drv_status_read_failures);
        return context->drv_status_read_failures >= STEPPER_DRV_STATUS_READ_FAILURES ? STEPPER_FAULT_DRIVER_COMM : 0;
    }
    context->drv_status_read_failures = 0;

    uint32_t fault = 0;
    if (reg.drv_status.otpw) {
        fault |= STEPPER_FAULT_OVERTEMP_WARNING;
    }
    if (reg.drv_status.ot) {
        fault |= STEPPER_FAULT_OVERTEMP;
    }

    return fault;
}

/**
 * Latch `fault`, stop stepping, disable the driver, and hand the move to the CAN layer as MOTION_FAULT so it is
 * acked with an error. What happens next is the host's call; see `stepper_clear_fault`.
 *
 * STEPPER_FAULT_MOVE_TIMEOUT is dropped unless the move whose `move_deadline_ms` was `deadline` is still running: a
 * move that ended, or gave way to another, meanwhile didn't time out. Thermal and communication faults stand.
 */
static void stepper_trip_fault(struct stepper_work_context *context, uint32_t fault, const int64_t deadline) {
    const uint8_t motor_id = ll_motor_get_id(context->dev);

    k_mutex_lock(&stepper_fault_lock, K_FOREVER);

    // Claim the move before stopping it, so a DMA or limit-switch event can't report it as done.
    const unsigned int key = irq_lock();
    if (context->motion_mode != MOTION_IN_PROGESS || context->move_deadline_ms != deadline) {
        fault &= ~STEPPER_FAULT_MOVE_TIMEOUT;
    }
    if (fault == 0) {
        irq_unlock(key);
        k_mutex_unlock(&stepper_fault_lock);
        return;
    }
    atomic_or(&context->fault, (atomic_val_t)fault);

    if (context->motion_mode == MOTION_IN_PROGESS) {
        context->motion_mode = MOTION_FAULT;
    }
    context->motion_calculation_done = true;
    // The carriage stopped short of `last_position_generated` (and homing never tracked it at all), so moves need a
    // successful home once the fault is cleared, as after an e-stop.
    atomic_flag_test_and_set(&context->e_stop_triggered);
    irq_unlock(key);

    // No more refills, then halt the pulse train and drop the queued blocks.
    struct k_work_sync sync;
    k_work_cancel_delayable_sync(&context->calculation_work, &sync);
    ll_stepper_abort(context->dev);

    const struct device *driver = stepper_driver_of(context);
    if (driver != NULL) {
        const int ret = adi_tmc2209_output_disable(driver);
        context->driver_disabled = ret == 0;
        if (ret < 0) {
            LOG_ERR("Stepper %d driver not confirmed disabled (%d); STEP output is stopped", motor_id, ret);
        }
    }
    k_mutex_unlock(&stepper_fault_lock);

    LOG_ERR("Stepper %d fault 0x%X: move aborted at %f rev (generated), driver %s", motor_id, fault,
            (double)context->context.last_position_generated, context->driver_disabled ? "disabled" : "NOT disabled");
}

static void stepper_work_check_driver_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct stepper_work_context *context = CONTAINER_OF(dwork, struct stepper_work_context, check_driver_work);

    // Only a moving stepper is watched; the next move reschedules this.
    if (context->motion_mode != MOTION_IN_PROGESS) {
        return;
    }

    // The read blocks, and the move can end or another start meanwhile; `stepper_trip_fault` checks that this
    // deadline still belongs to the running move.
    const int64_t deadline = context->move_deadline_ms;
    uint32_t fault = stepper_poll_driver(context);
    if (deadline != 0 && k_uptime_get() > deadline) {
        fault |= STEPPER_FAULT_MOVE_TIMEOUT;
    }
    if (context->homing_travel_exhausted) {
        context->homing_travel_exhausted = false;
        LOG_ERR("Stepper %d covered its maximum homing travel without reaching the limit switch",
                ll_motor_get_id(context->dev));
        fault |= STEPPER_FAULT_MOVE_TIMEOUT;
    }

    if (fault != 0) {
        stepper_trip_fault(context, fault, deadline);
        return;
    }

    k_work_reschedule_for_queue(&stepper_monitor_workq, dwork, K_MSEC(STEPPER_DRIVER_CHECK_PERIOD));
}

int stepper_clear_fault(const struct device *dev) {
    struct stepper_work_context *context = find_stepper_context_from_device(dev);
    if (context == NULL) {
        return -ENODEV;
    }

    const uint8_t motor_id = ll_motor_get_id(dev);
    int ret = 0;

    k_mutex_lock(&stepper_fault_lock, K_FOREVER);
    const uint32_t fault = (uint32_t)atomic_get(&context->fault);
    if (context->motion_mode == MOTION_IN_PROGESS) {
        ret = -EBUSY;
    } else if (fault != 0 || context->driver_disabled) {
        const struct device *driver = stepper_driver_of(context);
        if (driver != NULL) {
            adi_tmc2209_reg_t reg = {0};
            ret = adi_tmc2209_read_drv_status(driver, &reg);
            if (ret == 0 && (reg.drv_status.otpw || reg.drv_status.ot)) {
                LOG_WRN("Stepper %d driver still reports otpw %d ot %d", motor_id, reg.drv_status.otpw,
                        reg.drv_status.ot);
                ret = -EAGAIN;
            }
            if (ret == 0) {
                ret = adi_tmc2209_output_enable(driver);
            }
        }

        if (ret == 0) {
            context->driver_disabled = false;
            context->drv_status_read_failures = 0;
            atomic_clear(&context->fault);
        }
    }
    k_mutex_unlock(&stepper_fault_lock);

    if (ret < 0) {
        LOG_ERR("Stepper %d fault 0x%X not cleared: %d", motor_id, fault, ret);
    } else if (fault != 0) {
        LOG_INF("Stepper %d fault 0x%X cleared, driver enabled", motor_id, fault);
    }

    return ret;
}

uint32_t stepper_get_fault(const struct device *dev) {
    const struct stepper_work_context *context = find_stepper_context_from_device(dev);
    return context == NULL ? 0 : (uint32_t)atomic_get(&context->fault);
}

bool stepper_driver_output_disabled(const struct device *dev) {
    const struct stepper_work_context *context = find_stepper_context_from_device(dev);
    return context != NULL && context->driver_disabled;
}

int stepper_fault_error(const uint32_t fault) { return fault == STEPPER_FAULT_MOVE_TIMEOUT ? -ETIMEDOUT : -EIO; }

int stepper_motion_error(const struct device *dev) {
    const struct stepper_work_context *context = find_stepper_context_from_device(dev);
    if (context == NULL) {
        return -ENODEV;
    }

    return context->homing_error != 0 ? context->homing_error
                                      : stepper_fault_error((uint32_t)atomic_get(&context->fault));
}

/* ***** Initialization ***** */
static int motor_workq_init_and_start(void) {
    for (size_t i = 0; i < ARRAY_SIZE(stepper_contexts); i++) {
        struct stepper_work_context *context = &stepper_contexts[i];
        context->stepper_cb.user_data = context;
        const int ret = ll_stepper_register_callback(context->dev, &context->stepper_cb);
        if (ret < 0) {
            LOG_ERR("Error registering stepper callback: %d", ret);
        }
    }

    for (size_t i = 0; i < ARRAY_SIZE(servo_contexts); i++) {
        struct servo_work_context *context = &servo_contexts[i];
        context->servo_cb.user_data = context;
        const int ret = ll_servo_register_callback(context->dev, &context->servo_cb);
        if (ret < 0) {
            LOG_ERR("Error registering servo callback: %d", ret);
        }
    }

    k_work_queue_init(&motor_workq);

    for (size_t i = 0; i < ARRAY_SIZE(stepper_contexts); i++) {
        k_work_init_delayable(&stepper_contexts[i].calculation_work, stepper_work_calculation_handler);
        k_work_init_delayable(&stepper_contexts[i].check_driver_work, stepper_work_check_driver_handler);
        k_work_init_delayable(&stepper_contexts[i].homing_verify_work, stepper_work_homing_verify_handler);
    }

    for (size_t i = 0; i < ARRAY_SIZE(servo_contexts); i++) {
        k_work_init_delayable(&servo_contexts[i].calculation_work, servo_work_calculation_handler);
    }

    motor_settings_init();

    for (size_t i = 0; i < ARRAY_SIZE(stepper_contexts); i++) {
        struct stepper_work_context *context = &stepper_contexts[i];
        const struct device *motor_dev = context->dev;
        const ll_motor_cfg_t *motor_data = motor_dev->config;
        const struct device *stepper_driver_dev = motor_data->stepper_driver_device;
        if (stepper_driver_dev != NULL) {
            const int ret = adi_tmc2209_set_microstep(stepper_driver_dev, context->microsteps);
            if (ret < 0) {
                LOG_ERR("Stepper %u driver didn't take microsteps %u (%d); moves refused until reconfigured",
                        (unsigned int)i, context->microsteps, ret);
                context->microsteps_unverified = true;
            }
        }
    }

    k_work_queue_start(&motor_workq, motor_workq_stack, K_THREAD_STACK_SIZEOF(motor_workq_stack),
                       CONFIG_LIB_MOTOR_MOTION_WORK_QUEUE_PRIORITY, NULL);

#if DT_HAS_COMPAT_STATUS_OKAY(ll_stepper)
    k_work_queue_init(&stepper_monitor_workq);
    k_work_queue_start(&stepper_monitor_workq, stepper_monitor_workq_stack,
                       K_THREAD_STACK_SIZEOF(stepper_monitor_workq_stack),
                       CONFIG_LIB_MOTOR_MOTION_STEPPER_MONITOR_PRIORITY, NULL);
#endif
    return 0;
}

SYS_INIT(motor_workq_init_and_start, APPLICATION, 99);

/* ***** Initialize New Movements ***** */

#define UNCHANGED_UINT32 ((uint32_t) - 1)
int servo_set_parameters(const struct device *dev, const float max_velocity, const float max_acceleration,
                         const float min_angle_pwm, const float max_angle_pwm) {
    struct servo_work_context *const context = find_servo_context_from_device(dev);
    if (context == NULL) {
        return -ENODEV;
    }

    if (max_velocity > 0.0f) {
        context->motor_max_velocity = max_velocity;
    }

    if (max_acceleration > 0.0f) {
        context->motor_max_acceleration = max_acceleration;
    }

    if (min_angle_pwm > 0.0f) {
        context->context.min_angle_pwm = min_angle_pwm;
    }

    if (max_angle_pwm > 0.0f) {
        context->context.max_angle_pwm = max_angle_pwm;
    }

    motor_settings_save();

    return 0;
}

int servo_set_angle_parameters(const struct device *dev, const float min_angle, const float max_angle) {
    struct servo_work_context *const context = find_servo_context_from_device(dev);
    if (context == NULL) {
        return -ENODEV;
    }

    context->context.min_angle = min_angle;
    context->context.max_angle = max_angle;
    servo_assume_min_angle_position(context);
    return 0;
}

int servo_move_to_position(const struct device *dev, float target_position, const float max_velocity,
                           const float max_acceleration) {
    struct servo_work_context *context = find_servo_context_from_device(dev);
    /*
     * The library is currently not set up to allow servos to have limit switches
     * and homing. Thus, we do not check if the e-stop flag is set, because there
     * is no way to unset it.
     */

    if (context == NULL) {
        LOG_ERR("Stepper context not found for device");
        return -ENODEV;
    }

    if (context->motion_mode == MOTION_IN_PROGESS) {
        LOG_ERR("Attempted to move motor while already in motion.");
        return -EBUSY;
    }

    if (e_stop_engaged()) {
        LOG_ERR("Attempted to move servo while the e-stop is engaged");
        return -ECANCELED;
    }

    if (max_acceleration > context->motor_max_acceleration) {
        LOG_WRN("Max acceleration greater than that of the motor, using lower value.");
    }

    if (max_velocity > context->motor_max_velocity) {
        LOG_WRN("Max velocity greater than that of the motor, using lower value.");
    }

    const float movement_max_a = MIN(context->motor_max_acceleration, max_acceleration);
    const float movement_max_v = MIN(context->motor_max_velocity, max_velocity);

    // Limit position to within the desired angles
    if (target_position < context->context.min_angle) {
        target_position = context->context.min_angle;
    } else if (target_position > context->context.max_angle) {
        target_position = context->context.max_angle;
    }

    // Plan from the believed position. Bound it to the configured span so a host that narrows the limits after
    // a real position was established cannot make the profile start outside the PWM endpoints — but only when
    // the limits are usable: nothing validates them on either install path, and CLAMP with a non-finite or
    // inverted pair yields NAN or a value outside both.
    const float believed_position = context->context.last_position_generated;
    const float start_position = servo_angle_limits_usable(context)
                                     ? CLAMP(believed_position, context->context.min_angle, context->context.max_angle)
                                     : believed_position;

    const int ret = motor_motion_servo_init_context_struct(start_position, target_position, movement_max_v,
                                                           movement_max_a, context->context.min_angle_pwm,
                                                           context->context.max_angle_pwm, &context->context);

    if (ret != 0) {
        LOG_ERR("Failed to initialize context struct: %d", ret);
        return -EDOM;
    }

    // The generator owns `last_position_generated` from here, so the min-angle assumption is over.
    context->position_assumed = false;
    context->motion_mode = MOTION_IN_PROGESS;
    context->motion_calculation_done = false;

    // Start calculating the motion profile and load as many blocks as possible
    for (int i = 0; i < BUFS_PER_MOTOR; i++) {
        context->current_buffer = i;
        const ssize_t gen_table_ret = motor_motion_servo_generate_displacement_table(
            context->buffers[context->current_buffer], SERVO_BUFFER_SIZE, &context->context);
        context->last_calculation_ret = gen_table_ret;
        if (gen_table_ret <= 0) {
            LOG_ERR("Failed to generate servo table of size: %d", gen_table_ret);
            return -EDOM;
        }

        // Queue the buffer to the driver to start the motor motion
        ll_queue_servo_positions(context->dev, context->buffers[i], gen_table_ret * sizeof(uint32_t), K_FOREVER);

        // If the buffer wasn't full, then this is done and the next buffer isn't needed
        if (gen_table_ret < SERVO_BUFFER_SIZE) {
            context->motion_calculation_done = true;
            break;
        }
    }

    // Increment the buffer pointer
    context->current_buffer = (context->current_buffer + 1) % BUFS_PER_MOTOR;

    return 0;
}

int servo_move_relative(const struct device *dev, const float delta_position, const float max_velocity,
                        const float max_acceleration) {
    struct servo_work_context *context = find_servo_context_from_device(dev);
    return context == NULL ? -ENODEV
                           : servo_move_to_position(dev, context->context.last_position_generated + delta_position,
                                                    max_velocity, max_acceleration);
}

int stepper_set_parameters(const struct device *dev, const float max_velocity, const float max_acceleration,
                           const float homing_velocity, const uint16_t microsteps, const float steps_per_revolution,
                           const bool flip_limit_orientation) {
    struct stepper_work_context *const context = find_stepper_context_from_device(dev);
    if (context == NULL) {
        return -ENODEV;
    }

    // The queued step timings were planned with the current settings, and the limit-switch stop reads
    // `flip_limit_orientation`, so nothing may change underneath a move.
    if (context->motion_mode == MOTION_IN_PROGESS) {
        LOG_ERR("Refusing stepper configuration while the motor is moving");
        return -EBUSY;
    }

    // Validate everything before changing anything. A finite value <= 0 means "unchanged"; NaN and both
    // infinities are rejected, since comparisons would otherwise treat NaN and -Inf as "unchanged" too.
    if (!isfinite(max_velocity) || !isfinite(max_acceleration) || !isfinite(homing_velocity) ||
        !isfinite(steps_per_revolution)) {
        LOG_ERR("Refusing non-finite stepper configuration");
        return -EINVAL;
    }

    // The TMC2209 takes only powers of two up to 256.
    if (microsteps > 0 && (microsteps > 256 || (microsteps & (microsteps - 1)) != 0)) {
        LOG_ERR("Invalid microsteps %u: must be a power of two from 1 to 256", microsteps);
        return -EINVAL;
    }

    // Planner and driver must agree on the step size, so the planner takes the new value only once the driver has.
    if (microsteps > 0) {
        const struct device *driver = stepper_driver_of(context);
        if (driver != NULL) {
            const int ret = adi_tmc2209_set_microstep(driver, microsteps);
            if (ret < 0) {
                LOG_ERR("Driver didn't take microsteps %u (%d); keeping %u", microsteps, ret, context->microsteps);
                return ret;
            }
        }
        context->microsteps = microsteps;
        context->microsteps_unverified = false;
    }

    if (max_velocity > 0.0f) {
        context->motor_max_velocity = max_velocity;
    }

    if (max_acceleration > 0.0f) {
        context->motor_max_acceleration = max_acceleration;
    }

    if (homing_velocity > 0.0f) {
        context->homing_velocity = homing_velocity;
    }

    if (steps_per_revolution > 0.0f) {
        context->motor_steps_per_revolution = steps_per_revolution;
    }

    context->flip_limit_orientation = flip_limit_orientation != 0;  // ensure 0/1 result

    motor_settings_save();
    return 0;
}

int stepper_save_fixed_location(struct stepper_work_context *context, const int motor_id, const float position, const bool is_absolute) {
    if (!isfinite(position)) {
        LOG_ERR("Refusing non-finite fixed position");
        return -EINVAL;
    }

    if (is_absolute) {
        context->fixed_position = position;
    } else {
        float val = context->fixed_position + position;
        
        context->fixed_position = (val <= 0.0f) ? 0.0f : (val > 15.0f ? val = 15.0f : val);        
    }

    motor_settings_save();

    return 0;
}

int stepper_move_to_position(const struct device *dev, const float target_position, const float max_velocity,
                             const float max_acceleration) {
    struct stepper_work_context *context = find_stepper_context_from_device(dev);
    if (max_acceleration <= 0.0f || max_velocity <= 0.0f || isnan(max_acceleration) || isnan(max_velocity) ||
        isinf(max_acceleration) || isinf(max_velocity) || isnan(target_position) || isinf(target_position)) {
        LOG_ERR("Invalid paramaters: max_a: %f, max_v: %f, target: %f", (double)max_acceleration, (double)max_velocity,
                (double)target_position);
        return -EINVAL;
    }

    if (context == NULL) {
        LOG_ERR("Stepper context not found for device");
        return -ENODEV;
    }

    if (context->motion_mode == MOTION_IN_PROGESS) {
        LOG_ERR("Attempted to move while move is active");
        return -EBUSY;
    }

    if (e_stop_engaged()) {
        LOG_ERR("Attempted to move while the e-stop is engaged");
        return -ECANCELED;
    }

    if (atomic_get(&context->fault) != 0) {
        LOG_ERR("Attempted to move with fault 0x%lX latched", atomic_get(&context->fault));
        return -EPERM;
    }

    if (!stepper_driver_config_verified(context)) {
        LOG_ERR("Attempted to move with the driver configuration unverified");
        return -EIO;
    }

    if (atomic_flag_test_and_set(&context->e_stop_triggered)) {
        LOG_ERR("Attempted to move motor after e-stop, protection fault or failed homing without homing!");
        return -EBUSY;
    }
    atomic_flag_clear(&context->e_stop_triggered);

    // Each step pulse moves 1 / (microsteps * steps_per_revolution) position units; a shorter move has no pulses.
    const float pulses = fabsf(target_position - context->context.last_position_generated) *
                         context->motor_steps_per_revolution * (float)context->microsteps;
    if (pulses < 1.0f) {
        LOG_WRN("Target position is the same as current position.");
        return -EAGAIN;
    }

    context->homing_verify = HOMING_VERIFY_NONE;
    context->homing_error = 0;

    return stepper_start_move(context, target_position, max_velocity, max_acceleration);
}

/**
 * Start a move of `context`'s motor from `last_position_generated` to `target_position`. The caller has checked
 * that the motor may move; the homing verification calls this while homing is still MOTION_IN_PROGESS.
 */
static int stepper_start_move(struct stepper_work_context *context, const float target_position,
                              const float max_velocity, const float max_acceleration) {
    const struct device *dev = context->dev;

    if (target_position < context->context.last_position_generated) {
        context->motor_direction = context->flip_limit_orientation ? LL_STEPPER_DIR_FORWARD : LL_STEPPER_DIR_BACKWARD;
    } else {
        context->motor_direction = context->flip_limit_orientation ? LL_STEPPER_DIR_BACKWARD : LL_STEPPER_DIR_FORWARD;
    }

    ll_stepper_set_direction(dev, context->motor_direction);

    if (max_acceleration > context->motor_max_acceleration) {
        LOG_WRN("Max acceleration greater than that of the motor, using lower value.");
    }

    if (max_velocity > context->motor_max_velocity) {
        LOG_WRN("Max velocity greater than that of the motor, using lower value.");
    }

    const float movement_max_a = MIN(context->motor_max_acceleration, max_acceleration);
    float movement_max_v = MIN(context->motor_max_velocity, max_velocity);
    const float timer_max_v = stepper_max_timer_velocity(context);
    if (movement_max_v > timer_max_v) {
        LOG_WRN("Velocity %f is beyond what the step timer can produce at %u microsteps; using %f",
                (double)movement_max_v, context->microsteps, (double)timer_max_v);
        movement_max_v = timer_max_v;
    }
    const float distance = fabsf(target_position - context->context.last_position_generated);
    const int ret = motor_motion_stepper_init_context_struct(
        context->context.last_position_generated, target_position, movement_max_v, movement_max_a, context->microsteps,
        context->timer_increment, context->motor_steps_per_revolution, &context->context);

    if (ret != 0) {
        LOG_ERR("Failed to initialize context struct: %d", ret);
        return -EDOM;
    }

    // Set before MOTION_IN_PROGESS so a pending check can't judge this move by the last one's deadline.
    context->move_deadline_ms = k_uptime_get() + stepper_move_timeout_ms(distance, movement_max_v, movement_max_a);
    context->drv_status_read_failures = 0;
    context->homing_travel_exhausted = false;

    const motion_mode_t prev_motion_mode = context->motion_mode;
    context->motion_mode = MOTION_IN_PROGESS;
    context->move_control = MOVING_POSITION;
    context->motion_calculation_done = false;

    ll_stepper_enable(dev);

    // Start calculating the motion profile and load as many blocks as possible
    for (int i = 0; i < BUFS_PER_MOTOR; i++) {
        context->current_buffer = i;
        const ssize_t gen_table_ret =
            motor_motion_stepper_generate_timing_table(context->buffers[i], STEPPER_BUFFER_SIZE, &context->context);
        context->last_calculation_ret = gen_table_ret;

        // Nothing was queued, so no DMA event will ever end this move; don't leave it in progress.
        if (i == 0 && gen_table_ret <= 0) {
            LOG_ERR("Stepper move generated no pulses (%d)", gen_table_ret);
            context->motion_calculation_done = true;
            context->motion_mode = prev_motion_mode;
            return gen_table_ret < 0 ? gen_table_ret : -EAGAIN;
        }

        // Error out if calculation didn't succeed
        if (gen_table_ret < 0) {
            LOG_ERR("Error generating stepper table.");
            return gen_table_ret;
        }

        // Submit the buffer to the driver to start the motor motion
        if (gen_table_ret > 0) {
            ll_queue_stepper_positions(dev, context->buffers[i], gen_table_ret * sizeof(uint32_t), K_FOREVER);
        }

        // If the buffer wasn't full, then this is done and the next buffer isn't needed
        if (gen_table_ret < STEPPER_BUFFER_SIZE) {
            context->motion_calculation_done = true;
            break;
        }
    }

    // Increment the buffer pointer
    context->current_buffer = (context->current_buffer + 1) % BUFS_PER_MOTOR;

    stepper_monitor_start(context);

    return 0;
}

int stepper_move_relative(const struct device *dev, const float delta_position, const float max_velocity,
                          const float max_acceleration) {
    const struct stepper_work_context *context = find_stepper_context_from_device(dev);
    return context == NULL ? -ENODEV
                           : stepper_move_to_position(dev, context->context.last_position_generated + delta_position,
                                                      max_velocity, max_acceleration);
}

int stepper_home(const struct device *dev) {
    struct stepper_work_context *work_context = find_stepper_context_from_device(dev);
    stepper_motor_context_t *context = &work_context->context;
    if (context == NULL) {
        return -ENODEV;
    }

    const ll_motor_cfg_t *cfg = dev->config;

    if (e_stop_engaged()) {
        LOG_ERR("Attempted to home while the e-stop is engaged");
        return -ECANCELED;
    }

    if (atomic_get(&work_context->fault) != 0) {
        LOG_ERR("Attempted to home with fault 0x%lX latched", atomic_get(&work_context->fault));
        return -EPERM;
    }

    if (!stepper_driver_config_verified(work_context)) {
        LOG_ERR("Attempted to home with the driver configuration unverified");
        return -EIO;
    }

    if (cfg->limit_switch_pin.port == NULL) {
        LOG_ERR("Limit switch pin not set");
        return -ENOTSUP;
    }

    if (work_context->motion_mode == MOTION_IN_PROGESS) {
        LOG_ERR("Attempted to move motor while already in motion.");
        return -EBUSY;
    }

    atomic_flag_clear(&work_context->e_stop_triggered);
    work_context->homing_error = 0;
    work_context->homing_verify = HOMING_VERIFY_NONE;

    if (ll_stepper_get_limit_switch_state(dev) == 1) {
        // Could be a switch stuck active, so this is only home once the verification has seen it release.
        LOG_INF("Already touching limit switch; verifying it");
        // The verification moves set their own deadlines; there's nothing to time until the first starts.
        work_context->move_deadline_ms = 0;
        work_context->drv_status_read_failures = 0;
        work_context->homing_travel_exhausted = false;
        work_context->move_control = MOVING_HOME;
        work_context->homing_verify = HOMING_VERIFY_START;
        work_context->motion_mode = MOTION_IN_PROGESS;
        stepper_homing_at_switch(work_context);
        stepper_monitor_start(work_context);
        return 0;
    }

    context->min_step = 1.0f / work_context->microsteps;

    // Ramp up as a move does, over a displacement long enough to reach the homing velocity; after the ramp,
    // `stepper_generate_homing_block` holds that velocity until the limit switch. Homing heads toward position
    // 0, so the profile runs down from the current position.
    const float homing_v = MIN(work_context->homing_velocity, stepper_max_timer_velocity(work_context));
    const float homing_a = work_context->motor_max_acceleration;
    const float ramp_start = context->last_position_generated;
    const int ramp_ret = motor_motion_stepper_init_context_struct(
        ramp_start, ramp_start - 4.0f * homing_v * homing_v / homing_a, homing_v, homing_a, work_context->microsteps,
        work_context->timer_increment, work_context->motor_steps_per_revolution, context);
    work_context->homing_ramp_done = ramp_ret != 0;
    if (ramp_ret != 0) {
        LOG_WRN("Homing ramp unavailable (%d); homing at constant velocity", ramp_ret);
    }

    // Timed like a move across the longest homing travel. Homing doesn't ramp down, so this is a little more
    // generous than the move it's modelled on. Set before MOTION_IN_PROGESS so a pending check can't judge
    // homing by the last move's deadline.
    work_context->move_deadline_ms =
        k_uptime_get() +
        stepper_move_timeout_ms((float)CONFIG_LIB_MOTOR_MOTION_STEPPER_HOMING_MAX_TRAVEL, homing_v, homing_a);
    work_context->drv_status_read_failures = 0;
    // The pulses in the longest homing travel: position units * full steps per unit * pulses per full step.
    work_context->homing_pulses_left =
        (uint32_t)lroundf((float)CONFIG_LIB_MOTOR_MOTION_STEPPER_HOMING_MAX_TRAVEL *
                          work_context->motor_steps_per_revolution * (float)work_context->microsteps);
    work_context->homing_travel_exhausted = false;
    work_context->move_control = MOVING_HOME;
    work_context->motion_mode = MOTION_IN_PROGESS;
    work_context->motor_direction =
        work_context->flip_limit_orientation ? LL_STEPPER_DIR_FORWARD : LL_STEPPER_DIR_BACKWARD;
    work_context->motion_calculation_done = false;

    ll_stepper_set_direction(dev, work_context->motor_direction);

    ll_stepper_enable(dev);

    // Queue every buffer, as a move does, so each block completing triggers the next refill without a gap.
    for (int i = 0; i < BUFS_PER_MOTOR; i++) {
        const size_t ret = stepper_generate_homing_block(work_context, work_context->buffers[i]);
        work_context->last_calculation_ret = (ssize_t)ret;

        LOG_DBG("Q buf %d [%p]", i, (void *)work_context->buffers[i]);
        ll_queue_stepper_positions(dev, work_context->buffers[i], ret * sizeof(uint32_t), K_FOREVER);

        // The whole travel fit in this block; an empty one would confuse the DMA callbacks.
        if (work_context->motion_calculation_done) {
            break;
        }
    }

    // buffers[0] plays out first, so it is the first to refill.
    work_context->current_buffer = 0;

    stepper_monitor_start(work_context);

    return 0;
}

/**
 * Take the homing verification's next step, once the motor has stopped at the switch or a verification move has
 * ended. The moves are normal moves at the motor's maximum velocity and acceleration.
 */
static void stepper_work_homing_verify_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct stepper_work_context *context = CONTAINER_OF(dwork, struct stepper_work_context, homing_verify_work);
    const uint8_t motor_id = ll_motor_get_id(context->dev);

    if (context->homing_verify == HOMING_VERIFY_NONE) {
        return;
    }

    // A fault or a stop ended homing first; there's nothing left to verify.
    if (context->motion_mode != MOTION_IN_PROGESS) {
        stepper_homing_finish(context, 0);
        return;
    }

    int ret;
    switch (context->homing_verify) {
        case HOMING_VERIFY_START:
            context->homing_verify = HOMING_VERIFY_AWAY;
            ret = stepper_start_move(context, STEPPER_HOMING_VERIFY_DISTANCE, context->motor_max_velocity,
                                     context->motor_max_acceleration);
            break;

        case HOMING_VERIFY_AWAY:
            // A defective switch still gets the move back, so the carriage ends where homing left it; the error
            // is held until then.
            if (ll_stepper_get_limit_switch_state(context->dev) != 0) {
                LOG_ERR("Stepper %d limit switch still active %.1f from home; switch is defective", motor_id,
                        (double)STEPPER_HOMING_VERIFY_DISTANCE);
                context->homing_error = -ENXIO;
            }

            // A working switch closes on the way back, which stops the move and re-zeroes there. A defective one is
            // only taken back to 0, where homing left the carriage, rather than past it into the switch.
            context->homing_verify = HOMING_VERIFY_RETURN;
            ret = stepper_start_move(context, context->homing_error == 0 ? -STEPPER_HOMING_VERIFY_OVERTRAVEL : 0.0f,
                                     context->motor_max_velocity, context->motor_max_acceleration);
            break;

        case HOMING_VERIFY_RETURN:
            // The move back ended without the switch closing.
            if (context->homing_error == 0) {
                LOG_ERR("Stepper %d limit switch didn't close again within %.1f past home", motor_id,
                        (double)STEPPER_HOMING_VERIFY_OVERTRAVEL);
                context->homing_error = -ENXIO;
            }
            stepper_homing_finish(context, context->homing_error);
            return;

        case HOMING_VERIFY_CLOSED:
            if (context->homing_error == 0) {
                LOG_INF("Stepper %d homed; limit switch verified", motor_id);
            }
            stepper_homing_finish(context, context->homing_error);
            return;

        default:
            return;
    }

    if (ret < 0) {
        LOG_ERR("Stepper %d homing verification move failed to start: %d", motor_id, ret);
        context->motion_calculation_done = true;
        ll_stepper_abort(context->dev);
        stepper_homing_finish(context, ret);
    }
}

int servo_read_config(const struct device *dev, servo_config_t *config) {
    const struct servo_work_context *context = find_servo_context_from_device(dev);
    if (context == NULL) {
        return -ENODEV;
    }

    config->min_position = context->context.min_angle;
    config->max_position = context->context.max_angle;
    config->max_pwm_duration_us = context->context.max_angle_pwm;
    config->min_pwm_duration_us = context->context.min_angle_pwm;
    config->motor_max_acceleration = context->motor_max_acceleration;
    config->motor_max_velocity = context->motor_max_velocity;

    return 0;
}

void set_all_e_stop_flags(void) {
    for (size_t i = 0; i < ARRAY_SIZE(stepper_contexts); i++) {
        atomic_flag_test_and_set(&stepper_contexts[i].e_stop_triggered);
    }

    for (size_t i = 0; i < ARRAY_SIZE(servo_contexts); i++) {
        atomic_flag_test_and_set(&servo_contexts[i].e_stop_triggered);
    }
}

void stepper_e_stop(const struct device *dev) {
    struct stepper_work_context *context = find_stepper_context_from_device(dev);
    if (context == NULL) {
        return;
    }

    // Claim the move before stopping it, so a DMA or limit-switch event can't report it as done, and stop any
    // refill or homing verification step from starting another.
    const unsigned int key = irq_lock();
    context->motion_calculation_done = true;
    context->homing_verify = HOMING_VERIFY_NONE;
    if (context->motion_mode == MOTION_IN_PROGESS) {
        context->homing_error = -ECANCELED;
        context->motion_mode = MOTION_FAULT;
    }
    irq_unlock(key);

    ll_stepper_abort(dev);

    struct k_work_sync sync;
    k_work_cancel_delayable_sync(&context->calculation_work, &sync);
    k_work_cancel_delayable_sync(&context->homing_verify_work, &sync);
    // A refill or verification move that was already running may have queued a block and restarted the DMA.
    ll_stepper_abort(dev);

    // The carriage stopped short of wherever the move had planned to, so the position needs homing again.
    atomic_flag_test_and_set(&context->e_stop_triggered);
}

void servo_e_stop(const struct device *dev) {
    struct servo_work_context *context = find_servo_context_from_device(dev);
    if (context == NULL) {
        return;
    }

    const unsigned int key = irq_lock();
    context->motion_calculation_done = true;
    const bool was_moving = context->motion_mode == MOTION_IN_PROGESS;
    if (was_moving) {
        context->motion_mode = MOTION_FAULT;
    }
    irq_unlock(key);

    ll_servo_abort(dev);

    struct k_work_sync sync;
    k_work_cancel_delayable_sync(&context->calculation_work, &sync);
    // A refill that was already running may have queued a block and restarted the DMA.
    ll_servo_abort(dev);

    // The horn stopped somewhere inside the block that was playing, not at `known_position`, and
    // `last_position_generated` runs up to two blocks ahead of it. Plan the next move from the pulse width it holds,
    // or it would start by jumping to the generated position.
    context->position_valid = false;
    uint32_t count;
    if (was_moving && ll_servo_get_pulse_count(dev, &count) == 0) {
        const float position = motor_motion_servo_pwm_count_to_degrees(&context->context, count);
        if (isfinite(position)) {
            context->context.known_position = position;
            context->context.last_position_generated = position;
        }
    }
}

movement_control_t stepper_homing_status(const struct device *dev) {
    const struct stepper_work_context *context = find_stepper_context_from_device(dev);

    if (!context) {
        return MOVING_POSITION;
    }

    // The verification's moves are part of homing.
    return context->homing_verify != HOMING_VERIFY_NONE ? MOVING_HOME : context->move_control;
}

int stepper_read_config(const struct device *dev, struct stepper_config *config) {
    const struct stepper_work_context *context = find_stepper_context_from_device(dev);
    if (context == NULL) {
        return -ENODEV;
    }

    config->flip_limit_orientation = context->flip_limit_orientation;
    config->steps_per_revolution = context->motor_steps_per_revolution;
    config->microsteps = context->microsteps;
    config->motor_max_velocity = context->motor_max_velocity;
    config->motor_max_acceleration = context->motor_max_acceleration;
    config->homing_velocity = context->homing_velocity;

    return 0;
}
