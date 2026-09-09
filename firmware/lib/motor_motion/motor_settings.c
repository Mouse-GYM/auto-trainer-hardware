#include "motor_settings.h"

#include <math.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "motor_motion_workq.h"

LOG_MODULE_REGISTER(motor_settings);

// We have to use small names here because the nvs is not very large.
#define SETTINGS_MODULE_NAME "motor"
#define SETTINGS_SERVO_MODULE_NAME "ser"
#define SETTINGS_STEPPER_MODULE_NAME "ste"

// Common keys
#define MAX_VELOCITY_KEY "v_max"
#define MAX_ACCELERATION_KEY "a_max"

// Servo keys
#define MIN_ANGLE_KEY "o_min"
#define MAX_ANGLE_KEY "o_max"
#define ANGLE_ADJUSTMENT_KEY "o_adj"
#define POSITION_KEY "o_pos"
#define SERVO_MIN_ANGLE_PWM_KEY "pwm_min"
#define SERVO_MAX_ANGLE_PWM_KEY "pwm_max"

// Stepper keys
#define MICRO_STEP_KEY "s_mstep"
#define STEPS_PER_REVOLUTION_KEY "s_rev"
#define FLIP_LIMIT_ORIENTATION_KEY "s_flo"
#define FIXED_POSITION_KEY "s_fix"
#define HOME_VELOCITY_KEY "s_home"

K_THREAD_STACK_DEFINE(motor_settings_stack, CONFIG_LIB_MOTOR_MOTION_SETTINGS_WORK_QUEUE_STACK_SIZE);

static struct k_work_q motor_settings_work_q;
static struct k_work motor_settings_work;

void _motor_settings_save() { settings_save(); }

void motor_settings_save() { k_work_submit_to_queue(&motor_settings_work_q, &motor_settings_work); }

int motor_settings_init() {
    // A work queue thread is used to save settings in a low-priority context to
    // allow the rest of the system to function while the slow operation of
    // writing to flash can be performed in the background. The kernel work priority
    // is -1, and cannot be changed. to ensure background writes, a specific
    // queue is needed.
    k_work_queue_start(&motor_settings_work_q, motor_settings_stack, K_THREAD_STACK_SIZEOF(motor_settings_stack),
                       CONFIG_LIB_MOTOR_MOTION_SETTINGS_WORK_QUEUE_PRIORITY, NULL);

    k_work_init(&motor_settings_work, _motor_settings_save);

    LOG_INF("Initializing Settings.");
    int rc = settings_subsys_init();
    if (rc < 0) {
        LOG_ERR("settings_subsys_init: %d", rc);
    } else {
        rc = settings_load();
        if (rc < 0) {
            LOG_ERR("settings_load: %d", rc);
        } else {
            LOG_INF("Settings Loaded");
        }
    }

    return rc;
}

static __used float read_float(const char *name, const settings_read_cb read_cb, void *cb_arg, const bool validate,
                               float dflt) {
    float value;

    const int ret = read_cb(cb_arg, &value, sizeof(value));

    if (ret < 0) {
        LOG_ERR("Failed to read %s from settings: %d", name, ret);
        value = dflt;
    } else {
        LOG_DBG("restored %s: %f", name, (double)value);
    }

    return validate && (value <= 0.0f) ? dflt : value;
}

/*
 * For quantities where zero and negative values are legitimate, so `read_float`'s `validate` guard cannot be
 * used. Rejects a short record and a non-finite value as well, both of which `read_float` lets through.
 */
static __used float read_signed_float(const char *name, const settings_read_cb read_cb, void *cb_arg,
                                      const float dflt) {
    float value;

    const ssize_t ret = read_cb(cb_arg, &value, sizeof(value));

    if (ret != (ssize_t)sizeof(value) || !isfinite(value)) {
        LOG_WRN("Failed to read %s from settings: %d", name, (int)ret);
        return dflt;
    }

    LOG_DBG("restored %s: %f", name, (double)value);
    return value;
}

/* -------------------------------------------------------------------------- */

static __used uint16_t read_uint16(const char *name, const settings_read_cb read_cb, void *cb_arg, uint16_t dflt) {
    uint16_t value;

    const int ret = read_cb(cb_arg, &value, sizeof(value));

    if (ret < 0) {
        LOG_ERR("Failed to read %s from settings: %d", name, ret);
        value = dflt;
    } else {
        LOG_DBG("restored %s: %d", name, value);
    }

    return value;
}

/* -------------------------------------------------------------------------- */

static __used bool read_bool(const char *name, const settings_read_cb read_cb, void *cb_arg, bool dflt) {
    bool value;

    const int ret = read_cb(cb_arg, &value, sizeof(value));

    if (ret < 0) {
        LOG_ERR("Failed to read %s from settings: %d", name, ret);
        value = dflt;
    } else {
        LOG_DBG("restored %s: %d", name, value);
    }

    return value;
}

/* -------------------------------------------------------------------------- */

static __used int write_float(char *key, const char numeric, float value, const int index,
                              int (*storage_func)(const char *name, const void *value, size_t val_len)) {
    key[index] = numeric;
    const int ret = storage_func(key, &value, sizeof(value));
    if (ret < 0) {
        LOG_ERR("Failed to save %s: %d", key, ret);
    } else {
        LOG_DBG("Saved %s: %f", key, (double)value);
    }

    return ret;
}

/* -------------------------------------------------------------------------- */

static __used int write_uint16(char *key, const char numeric, const uint16_t value, const int index,
                               int (*storage_func)(const char *name, const void *value, size_t val_len)) {
    key[index] = numeric;
    const int ret = storage_func(key, &value, sizeof(value));
    if (ret < 0) {
        LOG_ERR("Failed to save %s: %d", key, ret);
    } else {
        LOG_DBG("Saved %s: %d", key, value);
    }
    return ret;
}

/* -------------------------------------------------------------------------- */

static __used int write_bool(char *key, const char numeric, const bool value, const int index,
                             int (*storage_func)(const char *name, const void *value, size_t val_len)) {
    key[index] = numeric;
    const int ret = storage_func(key, &value, sizeof(value));
    if (ret < 0) {
        LOG_ERR("Failed to save %s: %d", key, ret);
    } else {
        LOG_DBG("Saved %s: %d", key, value);
    }
    return ret;
}

/* -------------------------------------------------------------------------- */

#if CONFIG_DT_HAS_LL_SERVO_ENABLED
/* ***** Settings Handler ***** */
static int servo_settings_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
                              struct servo_work_context *context) {
    if (strncmp(key, MAX_VELOCITY_KEY, sizeof(MAX_VELOCITY_KEY) - 1) == 0) {
        context->motor_max_velocity = read_float("max_velocity", read_cb, cb_arg, true, 200);
    } else if (strncmp(key, MAX_ACCELERATION_KEY, sizeof(MAX_ACCELERATION_KEY) - 1) == 0) {
        context->motor_max_acceleration = read_float("max_acceleration", read_cb, cb_arg, true, 2000);
    } else if (strncmp(key, MIN_ANGLE_KEY, sizeof(MIN_ANGLE_KEY) - 1) == 0) {
        context->context.min_angle = read_float("min_angle", read_cb, cb_arg, true, 0);
    } else if (strncmp(key, MAX_ANGLE_KEY, sizeof(MAX_ANGLE_KEY) - 1) == 0) {
        context->context.max_angle = read_float("max_angle", read_cb, cb_arg, true, 120);
    } else if (strncmp(key, ANGLE_ADJUSTMENT_KEY, sizeof(ANGLE_ADJUSTMENT_KEY) - 1) == 0) {
        context->context.angle_adjustment = read_float("angle_adjustment", read_cb, cb_arg, true, 0);
    } else if (strncmp(key, SERVO_MIN_ANGLE_PWM_KEY, sizeof(SERVO_MIN_ANGLE_PWM_KEY) - 1) == 0) {
        context->context.min_angle_pwm = read_float("min_angle_pwm", read_cb, cb_arg, true, 1000);
    } else if (strncmp(key, SERVO_MAX_ANGLE_PWM_KEY, sizeof(SERVO_MAX_ANGLE_PWM_KEY) - 1) == 0) {
        context->context.max_angle_pwm = read_float("max_angle_pwm", read_cb, cb_arg, true, 2000);
    } else if (strncmp(key, POSITION_KEY, sizeof(POSITION_KEY) - 1) == 0) {
        // Staged, not applied: the angle limits this is validated against may load after it does, so the
        // decision belongs in the commit hook. NAN means there is nothing usable to apply.
        context->pending_position = read_signed_float("position", read_cb, cb_arg, NAN);
    } else {
        LOG_WRN("Unknown key: %s", key);
        return -EINVAL;
    }

    return 0;
}

static char itoa(const size_t value) { return (value > 9) ? '\0' : value + '0'; }

#define GENERATE_SERVO_TEMPLATE(name) SETTINGS_MODULE_NAME "/" SETTINGS_SERVO_MODULE_NAME "/0/" name

int servo_settings_export(const struct device *dev, const size_t dt_id,
                          int (*storage_func)(const char *name, const void *value, size_t val_len)) {
    const struct servo_work_context *const context = find_servo_context_from_device(dev);
    static const size_t id_index = sizeof(SETTINGS_MODULE_NAME "/" SETTINGS_SERVO_MODULE_NAME "/") - 1;
    int rc = 0;

    if (context == NULL) {
        LOG_ERR("No context found for the stepper device.");
        rc = -ENODEV;
    }

    const char numeric = itoa(dt_id);

    if (rc == 0) {
        static char key[] = GENERATE_SERVO_TEMPLATE(MAX_VELOCITY_KEY);
        rc = write_float(key, numeric, context->motor_max_velocity, id_index, storage_func);
    }

    if (rc == 0) {
        static char key[] = GENERATE_SERVO_TEMPLATE(MAX_ACCELERATION_KEY);
        rc = write_float(key, numeric, context->motor_max_acceleration, id_index, storage_func);
    }

    if (rc == 0) {
        static char key[] = GENERATE_SERVO_TEMPLATE(MIN_ANGLE_KEY);
        rc = write_float(key, numeric, context->context.min_angle, id_index, storage_func);
    }

    if (rc == 0) {
        static char key[] = GENERATE_SERVO_TEMPLATE(MAX_ANGLE_KEY);
        rc = write_float(key, numeric, context->context.max_angle, id_index, storage_func);
    }

    if (rc == 0) {
        static char key[] = GENERATE_SERVO_TEMPLATE(ANGLE_ADJUSTMENT_KEY);
        rc = write_float(key, numeric, context->context.angle_adjustment, id_index, storage_func);
    }

    if (rc == 0) {
        static char key[] = GENERATE_SERVO_TEMPLATE(SERVO_MIN_ANGLE_PWM_KEY);
        rc = write_float(key, numeric, context->context.min_angle_pwm, id_index, storage_func);
    }

    if (rc == 0) {
        static char key[] = GENERATE_SERVO_TEMPLATE(SERVO_MAX_ANGLE_PWM_KEY);
        rc = write_float(key, numeric, context->context.max_angle_pwm, id_index, storage_func);
    }

    if (rc == 0 && isfinite(context->persisted_position)) {
        static char key[] = GENERATE_SERVO_TEMPLATE(POSITION_KEY);
        rc = write_float(key, numeric, context->persisted_position, id_index, storage_func);
    }

    return rc;
}

static int servo_settings_commit(struct servo_work_context *const context) {
    if (context == NULL) {
        return 0;
    }

    const float candidate = context->pending_position;
    context->pending_position = NAN;

    // Judgeable against the configured travel, and inside it. An unusable pair of limits is not a licence to
    // believe the record: with `max_angle` at +Inf every finite candidate would pass a bare range test.
    const bool acceptable = isfinite(candidate) && servo_angle_limits_usable(context) &&
                            candidate >= context->context.min_angle && candidate <= context->context.max_angle;

    if (acceptable) {
        context->context.known_position = candidate;
        context->context.last_position_generated = candidate;
        context->persisted_position = candidate;
        context->position_assumed = false;
    } else {
        if (isfinite(candidate)) {
            LOG_WRN("Stored servo position %f rejected against limits [%f, %f]; assuming the minimum",
                    (double)candidate, (double)context->context.min_angle, (double)context->context.max_angle);
        }
        servo_assume_min_angle_position(context);
    }

    return 0;
}

#define DEFINE_SERVO_DEVICE_SETTINGS_FUNCTION(id)                                                            \
    static int servo_settings_set##id(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg) { \
        const struct device *dev = DEVICE_DT_GET(DT_INST(id, ll_servo));                                     \
        struct servo_work_context *context = find_servo_context_from_device(dev);                            \
        return servo_settings_set(key, len, read_cb, cb_arg, context);                                       \
    }

#define DEFINE_SERVO_DEVICE_SETTINGS_EXPORT_FUNCTION(id)                                                             \
    static int servo_settings_export##id(int (*storage_func)(const char *name, const void *value, size_t val_len)) { \
        const struct device *dev = DEVICE_DT_GET(DT_INST(id, ll_servo));                                             \
        return servo_settings_export(dev, id, storage_func);                                                         \
    }

#define DEFINE_SERVO_DEVICE_SETTINGS_COMMIT_FUNCTION(id)                   \
    static int servo_settings_commit##id(void) {                           \
        const struct device *dev = DEVICE_DT_GET(DT_INST(id, ll_servo));   \
        return servo_settings_commit(find_servo_context_from_device(dev)); \
    }

#define DEFINE_SERVO_DEVICE_SETTINGS_HANDLERS(id)                                                                \
    SETTINGS_STATIC_HANDLER_DEFINE(servo##id, SETTINGS_MODULE_NAME "/" SETTINGS_SERVO_MODULE_NAME "/" #id, NULL, \
                                   servo_settings_set##id, servo_settings_commit##id, servo_settings_export##id);

DT_FOREACH_OKAY_INST_ll_servo(DEFINE_SERVO_DEVICE_SETTINGS_FUNCTION);
DT_FOREACH_OKAY_INST_ll_servo(DEFINE_SERVO_DEVICE_SETTINGS_EXPORT_FUNCTION);
DT_FOREACH_OKAY_INST_ll_servo(DEFINE_SERVO_DEVICE_SETTINGS_COMMIT_FUNCTION);

DT_FOREACH_OKAY_INST_ll_servo(DEFINE_SERVO_DEVICE_SETTINGS_HANDLERS);
#endif

#if CONFIG_DT_HAS_LL_STEPPER_ENABLED
static int stepper_settings_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
                                struct stepper_work_context *context) {
    if (strncmp(key, MAX_VELOCITY_KEY, sizeof(MAX_VELOCITY_KEY) - 1) == 0) {
        context->motor_max_velocity = read_float("max_velocity", read_cb, cb_arg, true, 48);
    } else if (strncmp(key, MAX_ACCELERATION_KEY, sizeof(MAX_ACCELERATION_KEY) - 1) == 0) {
        context->motor_max_acceleration = read_float("max_acceleration", read_cb, cb_arg, true, 2400);
    } else if (strncmp(key, HOME_VELOCITY_KEY, sizeof(HOME_VELOCITY_KEY) - 1) == 0) {
        context->homing_velocity = read_float("homing_velocity", read_cb, cb_arg, true, 20);
    } else if (strncmp(key, MICRO_STEP_KEY, sizeof(MICRO_STEP_KEY) - 1) == 0) {
        context->microsteps = read_uint16("min_step", read_cb, cb_arg, 8);
    } else if (strncmp(key, STEPS_PER_REVOLUTION_KEY, sizeof(STEPS_PER_REVOLUTION_KEY) - 1) == 0) {
        context->motor_steps_per_revolution = read_float("steps_per_revolution", read_cb, cb_arg, true, 48);
    } else if (strncmp(key, FLIP_LIMIT_ORIENTATION_KEY, sizeof(FLIP_LIMIT_ORIENTATION_KEY) - 1) == 0) {
        context->flip_limit_orientation = read_bool("flip_limit_orientation", read_cb, cb_arg, false);
    } else if (strncmp(key, FIXED_POSITION_KEY, sizeof(FIXED_POSITION_KEY) - 1) == 0) {
        context->fixed_position = read_float("fixed", read_cb, cb_arg, true, 0);
    } else {
        LOG_WRN("Unknown key: %s", key);
        return -EINVAL;
    }
    return 0;
}

#define GENERATE_STEPPER_TEMPLATE(name) SETTINGS_MODULE_NAME "/" SETTINGS_STEPPER_MODULE_NAME "/0/" name

static int stepper_settings_export(const struct device *dev, const size_t dt_id,
                                   int (*storage_func)(const char *name, const void *value, size_t val_len)) {
    int rc = 0;
    const struct stepper_work_context *context = find_stepper_context_from_device(dev);
    static const size_t id_index = sizeof(SETTINGS_MODULE_NAME "/" SETTINGS_SERVO_MODULE_NAME "/") - 1;

    if (context == NULL) {
        LOG_ERR("No context found for the stepper device.");
        rc = -ENODEV;
    }

    const char numeric = itoa(dt_id);

    if (rc == 0) {
        static char key[] = GENERATE_STEPPER_TEMPLATE(MAX_VELOCITY_KEY);
        rc = write_float(key, numeric, context->motor_max_velocity, id_index, storage_func);
    }

    if (rc == 0) {
        static char key[] = GENERATE_STEPPER_TEMPLATE(MAX_ACCELERATION_KEY);
        rc = write_float(key, numeric, context->motor_max_acceleration, id_index, storage_func);
    }

    if (rc == 0) {
        static char key[] = GENERATE_STEPPER_TEMPLATE(HOME_VELOCITY_KEY);
        rc = write_float(key, numeric, context->homing_velocity, id_index, storage_func);
    }

    if (rc == 0) {
        static char key[] = GENERATE_STEPPER_TEMPLATE(MICRO_STEP_KEY);
        rc = write_uint16(key, numeric, context->microsteps, id_index, storage_func);
    }

    if (rc == 0) {
        static char key[] = GENERATE_STEPPER_TEMPLATE(STEPS_PER_REVOLUTION_KEY);
        rc = write_float(key, numeric, context->motor_steps_per_revolution, id_index, storage_func);
    }

    if (rc == 0) {
        static char key[] = GENERATE_STEPPER_TEMPLATE(FLIP_LIMIT_ORIENTATION_KEY);
        rc = write_bool(key, numeric, context->flip_limit_orientation, id_index, storage_func);
    }

    if (rc == 0) {
        static char key[] = GENERATE_STEPPER_TEMPLATE(FIXED_POSITION_KEY);
        rc = write_float(key, numeric, context->fixed_position, id_index, storage_func);
    }

    return rc;
}

#define DEFINE_STEPPER_DEVICE_SETTINGS_FUNCTION(id)                                                            \
    static int stepper_settings_set##id(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg) { \
        const struct device *dev = DEVICE_DT_GET(DT_INST(id, ll_stepper));                                     \
        struct stepper_work_context *context = find_stepper_context_from_device(dev);                          \
        return stepper_settings_set(key, len, read_cb, cb_arg, context);                                       \
    }

#define DEFINE_STEPPER_DEVICE_SETTINGS_EXPORT_FUNCTION(id)                                                             \
    static int stepper_settings_export##id(int (*storage_func)(const char *name, const void *value, size_t val_len)) { \
        const struct device *dev = DEVICE_DT_GET(DT_INST(id, ll_stepper));                                             \
        return stepper_settings_export(dev, id, storage_func);                                                         \
    }

#define DEFINE_STEPPER_DEVICE_SETTINGS_HANDLERS(id)                                                                  \
    SETTINGS_STATIC_HANDLER_DEFINE(stepper##id, SETTINGS_MODULE_NAME "/" SETTINGS_STEPPER_MODULE_NAME "/" #id, NULL, \
                                   stepper_settings_set##id, NULL, stepper_settings_export##id);

DT_FOREACH_OKAY_INST_ll_stepper(DEFINE_STEPPER_DEVICE_SETTINGS_FUNCTION);
DT_FOREACH_OKAY_INST_ll_stepper(DEFINE_STEPPER_DEVICE_SETTINGS_EXPORT_FUNCTION);

DT_FOREACH_OKAY_INST_ll_stepper(DEFINE_STEPPER_DEVICE_SETTINGS_HANDLERS);
#endif
