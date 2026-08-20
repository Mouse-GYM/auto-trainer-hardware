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

// Period between successive status checks of the stepper drivers
#define STEPPER_DRIVER_CHECK_PERIOD 100U
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
             .last_position_generated = 0.0f,                                                \
             .known_position = 0.0f,                                                         \
         },                                                                                  \
     .buffers = {{0}},                                                                       \
     .current_buffer = 0,                                                                    \
     .last_calculation_ret = 0,                                                              \
     .e_stop_triggered = {.__val = 0},                                                       \
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

void stepper_set_position_to_zero(const struct device *dev) {
    struct stepper_work_context *context = find_stepper_context_from_device(dev);
    if (context == NULL) {
        LOG_ERR("Stepper context not found for device");
        return;
    }

    motor_motion_stepper_set_current_position(&context->context, 0.0f);
    context->motion_mode = MOTION_DONE;
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
                            context->motion_mode = MOTION_DONE;
                        }
                        break;

                    case MOVING_HOME:
                        if (!context->motion_calculation_done) {
                            k_work_schedule_for_queue(&motor_workq, &context->calculation_work, K_NO_WAIT);
                        }
                        break;
                }
            }
            break;
        case LL_MOTOR_EVENT_LIMIT_SWITCH:
            // Context is NULL during this event because of limitations of the GPIO driver
            context = find_stepper_context_from_device(dev);
            if (context && context->motion_mode == MOTION_IN_PROGESS) {
                switch (context->move_control) {
                    case MOVING_HOME:
                        LOG_WRN("Found Limit Switch. Stopping Motor.");
                        stepper_motor_stop(dev);
                        stepper_set_position_to_zero(dev);
                        break;

                    default:
                    case MOVING_POSITION:
                        if ((context->motor_direction == LL_STEPPER_DIR_BACKWARD && !context->flip_limit_orientation) ||
                            (context->motor_direction == LL_STEPPER_DIR_FORWARD && context->flip_limit_orientation)) {
                            LOG_WRN("Found Limit Switch. Stopping Motor.");
                            stepper_motor_stop(dev);
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
    if (context == NULL) {
        return;
    }

    context->motion_mode = MOTION_DONE;
    k_work_cancel_delayable(&context->calculation_work);
    k_work_cancel_delayable(&context->check_driver_work);
}

void servo_cancel_all_work(const struct device *dev) {
    struct servo_work_context *context = find_servo_context_from_device(dev);
    k_work_cancel_delayable(&context->calculation_work);
    context->motion_mode = MOTION_DONE;
    context->motion_calculation_done = true;
    context->context.known_position = context->context.last_position_generated;
}

static void servo_work_calculation_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct servo_work_context *context = CONTAINER_OF(dwork, struct servo_work_context, calculation_work);

    const ssize_t ret = motor_motion_servo_generate_displacement_table(context->buffers[context->current_buffer],
                                                                       SERVO_BUFFER_SIZE, &context->context);
    context->last_calculation_ret = ret;
    if (ret < 0) {
        LOG_ERR("Error generating servo table: %d", ret);
        return;
    }

    // Entire buffer wasn't needed which indicates all the steps have been planned and no more calculations are required
    // If nothing was calculated, then the motor is already at the target position
    if (ret < STEPPER_BUFFER_SIZE) {
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

    const float seconds_per_pulse =
        1 / context->homing_velocity / context->motor_steps_per_revolution * context->context.min_step;

    size_t n_pulses = (size_t)floorf(slow_pulses_ms / 1000.0f / seconds_per_pulse);

    if (n_pulses > STEPPER_BUFFER_SIZE) {
        n_pulses = STEPPER_BUFFER_SIZE;
    }

    for (size_t i = 0; i < n_pulses; i++) {
        buf[i] = lroundf(seconds_per_pulse / context->timer_increment);
    }

    return n_pulses;
}

static void stepper_work_calculation_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct stepper_work_context *context = CONTAINER_OF(dwork, struct stepper_work_context, calculation_work);

    // Cancellation is asynchronous, so a handler that was already running must stop itself.
    if (context->motion_mode != MOTION_IN_PROGESS) {
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
            if (ret > 0 && context->motion_mode == MOTION_IN_PROGESS) {
                // Sending a buffer of length 0 here causes the event callbacks
                // to function a little weirdly.
                ll_queue_stepper_positions(context->dev, context->buffers[context->current_buffer],
                                           context->last_calculation_ret * sizeof(uint32_t), K_FOREVER);
            }

            // Increment the buffer pointer
            context->current_buffer = (context->current_buffer + 1) % BUFS_PER_MOTOR;
        } break;

        case MOVING_HOME: {
            const size_t ret = stepper_generate_table_for_homing(context, context->buffers[context->current_buffer]);
            context->last_calculation_ret = ret;

            LOG_DBG("Q buf %d [%p]", context->current_buffer, (void *)context->buffers[context->current_buffer]);
            if (context->motion_mode == MOTION_IN_PROGESS) {
                ll_queue_stepper_positions(context->dev, context->buffers[context->current_buffer],
                                           context->last_calculation_ret * sizeof(uint32_t), K_FOREVER);
            }

            context->current_buffer = (context->current_buffer + 1) % BUFS_PER_MOTOR;
        } break;
    }
}

static void stepper_work_check_driver_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    // struct stepper_work_context *context = CONTAINER_OF(dwork, struct stepper_work_context, check_driver_work);

    // TODO: IMPLEMENT

    k_work_reschedule_for_queue(&motor_workq, dwork, K_MSEC(STEPPER_DRIVER_CHECK_PERIOD));
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
            adi_tmc2209_set_microstep(stepper_driver_dev, context->microsteps);
        }
    }

    k_work_queue_start(&motor_workq, motor_workq_stack, K_THREAD_STACK_SIZEOF(motor_workq_stack),
                       CONFIG_LIB_MOTOR_MOTION_WORK_QUEUE_PRIORITY, NULL);
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

    const int ret = motor_motion_servo_init_context_struct(
        context->context.last_position_generated, target_position, movement_max_v, movement_max_a,
        context->context.min_angle_pwm, context->context.max_angle_pwm, &context->context);

    if (ret != 0) {
        LOG_ERR("Failed to initialize context struct: %d", ret);
        return -EDOM;
    }

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

    if (max_velocity > 0.0f) {
        context->motor_max_velocity = max_velocity;
    }

    if (max_acceleration > 0.0f) {
        context->motor_max_acceleration = max_acceleration;
    }

    if (homing_velocity > 0.0f) {
        context->homing_velocity = homing_velocity;
    }

    if (microsteps > 0) {
        context->microsteps = microsteps;
        const ll_motor_cfg_t *stepper_config = dev->config;
        adi_tmc2209_set_microstep(stepper_config->stepper_driver_device, microsteps);
    }

    if (steps_per_revolution > 0.0f) {
        context->motor_steps_per_revolution = steps_per_revolution;
    }

    context->flip_limit_orientation = flip_limit_orientation != 0;  // ensure 0/1 result

    motor_settings_save();
    return 0;
}

int stepper_save_fixed_location(struct stepper_work_context *context, const int motor_id, const float position, const bool is_absolute) {
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

    if (atomic_flag_test_and_set(&context->e_stop_triggered)) {
        LOG_ERR("Attempted to move motor after e-stop without homing!");
        return -EBUSY;
    }
    atomic_flag_clear(&context->e_stop_triggered);

    if (fabsf(target_position - context->context.last_position_generated) < 1.0f / context->microsteps) {
        LOG_WRN("Target position is the same as current position.");
        return -EAGAIN;
    }

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
    const float movement_max_v = MIN(context->motor_max_velocity, max_velocity);
    const int ret = motor_motion_stepper_init_context_struct(
        context->context.last_position_generated, target_position, movement_max_v, movement_max_a, context->microsteps,
        context->timer_increment, context->motor_steps_per_revolution, &context->context);

    if (ret != 0) {
        LOG_ERR("Failed to initialize context struct: %d", ret);
        return -EDOM;
    }

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

    if (cfg->limit_switch_pin.port == NULL) {
        LOG_ERR("Limit switch pin not set");
        return -ENOTSUP;
    }

    if (ll_stepper_get_limit_switch_state(dev) == 1) {
        LOG_INF("Already touching limit switch");
        stepper_set_position_to_zero(dev);
        return 0;
    }

    if (work_context->motion_mode == MOTION_IN_PROGESS) {
        LOG_ERR("Attempted to move motor while already in motion.");
        return -EBUSY;
    }

    atomic_flag_clear(&work_context->e_stop_triggered);

    context->min_step = 1.0f / work_context->microsteps;
    work_context->move_control = MOVING_HOME;
    work_context->motion_mode = MOTION_IN_PROGESS;
    work_context->motor_direction =
        work_context->flip_limit_orientation ? LL_STEPPER_DIR_FORWARD : LL_STEPPER_DIR_BACKWARD;
    work_context->motion_calculation_done = false;

    ll_stepper_set_direction(dev, work_context->motor_direction);

    ll_stepper_enable(dev);

    const size_t ret = stepper_generate_table_for_homing(work_context, work_context->buffers[0]);
    work_context->last_calculation_ret = (ssize_t)ret;

    LOG_DBG("Q buf %d [%p]", work_context->current_buffer, (void *)work_context->buffers[0]);
    ll_queue_stepper_positions(dev, work_context->buffers[0], work_context->last_calculation_ret * sizeof(uint32_t),
                               K_FOREVER);

    work_context->current_buffer = 1 % BUFS_PER_MOTOR;
    return 0;
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
        atomic_flag_test_and_set(&stepper_contexts[i].e_stop_triggered);
    }
}

movement_control_t stepper_homing_status(const struct device *dev) {
    const struct stepper_work_context *context = find_stepper_context_from_device(dev);

    return !context ? MOVING_POSITION : context->move_control;
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
