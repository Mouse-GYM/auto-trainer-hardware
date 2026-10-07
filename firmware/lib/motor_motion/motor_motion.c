#include "motor_motion.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <zephyr/logging/log.h>

#include "servo.h"
#include "stepper.h"

LOG_MODULE_REGISTER(motor_motion);

void motor_motion_stepper_set_current_position(stepper_motor_context_t *context, const float position) {
    context->motion_profile.end_pos = position;
    context->last_position_generated = position;
}

void motor_motion_servo_set_current_position(servo_motor_context_t *context, const float position) {
    context->angle_adjustment = context->motion_profile.start_pos - position;
}

void stepper_motor_stop(const struct device *dev) {
    stepper_cancel_all_work(dev);
    ll_stepper_disable(dev);
}

void servo_motor_stop(const struct device *dev) { ll_servo_enable(dev, false); }

#define DT_GET_COMMA(id) DEVICE_DT_GET(id),
static const struct device *const stepper_motors[] = {DT_FOREACH_STATUS_OKAY(ll_stepper, DT_GET_COMMA)};
static const struct device *const servo_motors[] = {DT_FOREACH_STATUS_OKAY(ll_servo, DT_GET_COMMA)};

void motors_all_stop(void) {
    for (size_t i = 0; i < ARRAY_SIZE(stepper_motors); i++) {
        stepper_motor_stop(stepper_motors[i]);
    }

    for (size_t i = 0; i < ARRAY_SIZE(servo_motors); i++) {
        servo_motor_stop(stepper_motors[i]);
    }
}

void trigger_e_stop(void) {
    motors_all_stop();

    set_all_e_stop_flags();
}

const struct device *stepper_motor_by_id(const size_t id) {
    if (id >= ARRAY_SIZE(stepper_motors)) {
        LOG_ERR("Invalid stepper motor id: %d", id);
        return NULL;
    }

    return stepper_motors[id];
}

const struct device *servo_motor_by_id(const size_t id) {
    if (id >= ARRAY_SIZE(servo_motors)) {
        LOG_ERR("Invalid servo motor id: %d", id);
        return NULL;
    }

    return servo_motors[id];
}
