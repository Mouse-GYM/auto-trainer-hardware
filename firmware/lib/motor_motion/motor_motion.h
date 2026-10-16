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

/*
 * Stop the motor by stopping the timer and the DMA. This does not affect the internal state of the motion library.
 */
void stepper_motor_stop(const struct device *dev);
void servo_motor_stop(const struct device *dev);

/*
 * Stop all motors! Uses a static list of all motors to stop them, so is very fast.
 */
void motors_all_stop(void);

/*
 * Engage the emergency stop (e-stop): every stepper and servo stops where it is (`stepper_e_stop`,
 * `servo_e_stop`), and moves and homing are refused with -ECANCELED until `release_e_stop`. Steppers then also
 * need homing before they move again. Blocks briefly for running work items; don't call from an ISR.
 */
void trigger_e_stop(void);

/*
 * Engage the e-stop latch alone: new moves and homing are refused from now on, but nothing that is running stops
 * until `trigger_e_stop`. Safe to call from ISRs.
 */
void latch_e_stop(void);

/*
 * Release the e-stop, so servos can move and steppers can home again.
 */
void release_e_stop(void);

/*
 * @return whether the e-stop is engaged. Safe to call from ISRs.
 */
bool e_stop_engaged(void);

/*
 * Get the stepper motor device by its ID. This is useful for the shell commands and CAN commands.
 */
const struct device *stepper_motor_by_id(size_t id);

/*
 * Get the servo motor device by its ID. This is useful for the shell commands and CAN commands.
 */
const struct device *servo_motor_by_id(size_t id);
