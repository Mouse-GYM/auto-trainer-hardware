#pragma once

#include "motor_math.h"
#include "motor_motion_workq.h"

/* ***** Helper Functions ***** */

/*
 * Set the current position of the stepper motor. Useful especially when the motor is at 'home'
 * to set this position as zero without continuing to move the motor.
 */
void motor_motion_stepper_set_current_position(stepper_motor_context_t *context, float position);

/*
 * Set the current position of the servo motor. It does this by setting the angle_adjustment.
 */
void motor_motion_servo_set_current_position(servo_motor_context_t *context, float position);

/* Stop the stepper by cancelling pending motion work, then stopping its timer and DMA. */
void stepper_motor_stop(const struct device *dev);

/* Stop servo pulse generation. */
void servo_motor_stop(const struct device *dev);

/*
 * Stop all motors! Uses a static list of all motors to stop them, so is very fast.
 */
void motors_all_stop(void);

/*
 * Trigger the emergency stop (e-stop) functionality. Calls `motors_all_stop` and
 * sets the flags in the relevant context structs.
 */
void trigger_e_stop(void);

/*
 * Get the stepper motor device by its ID. This is useful for the shell commands and CAN commands.
 */
const struct device *stepper_motor_by_id(size_t id);

/*
 * Get the servo motor device by its ID. This is useful for the shell commands and CAN commands.
 */
const struct device *servo_motor_by_id(size_t id);
