#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#ifndef M_PI
#define M_PI 3.1415927f
#endif

// Highest angle any servo may be configured or commanded to. The `max_angle` default, the settings fallback
// and the calibration validation all derive from this one value.
#define SERVO_MAX_ALLOWED_ANGLE 180.0f

// A pulse this long fills the whole PWM period, so it cannot be produced: the driver sets ARR to
// `SERVO_TIMER_PERIOD_COUNTS` (`firmware/drivers/motor/servo.c`) and a CCR above ARR never compares. Stated
// in microseconds because that is what the calibration carries; the count-domain equivalence is asserted in
// `motor_motion_workq.c`, which can see both this and the driver header.
#define SERVO_MAX_PULSE_DURATION_US 20000.0f

typedef struct motor_motion_profile {
    /* Parameters from the paper:
     * https://studylib.net/doc/8267759/sinusoidal-velocity-profiles-for-motion-control
     *
     * The units of `start` and `end` for a stepper are in signed revolutions from the 0 point.
     * The units of `start` and `end` for a servo are in degrees.
     *
     * `max_velocity` and `max_acceleration` for a stepper are revolutions per second and revolutions per second
     * squared. `max_velocity` and `max_acceleration` for a servo are in degrees per second and degrees per second
     * squared respectively.
     */

    float start_pos;
    float end_pos;
    float a_max;
    float v_max;
    float sgn;
    float y_f;    // end position, unsigned
    float y_s;    // half-way position
    float y_a;    // position when acceleration ends
    float v_w;    // velocity of the middle, 'velocity' region of the curve
    float t_o;    // half duration of the 'acceleration' region
    float t_a;    // duration of the acceleration region (and deceleration region)
    float omega;  // angular velocity of the cosine term in the acceleration & deceleration regions
    float k_s;    // positional scaling factor for the cosine term
    float t_k;    // duration of the velocity region
    float t_s;    // half-way time of the velocity region (and the total motion profile)
    float t_t;    // total time of the motion profile
} motor_motion_profile_t;

typedef struct servo_motor_context {
    motor_motion_profile_t motion_profile;
    // Servo-specific parameters
    float min_angle;         // Nominal minimum angle (degrees)
    float max_angle;         // Nominal maximum angle (degrees)
    float angle_adjustment;  // Actual angle at `min_angle_pwm`. Due to inter-servo
                             // variations, the time for the "minumum angle" is sometimes different
                             // from the actual nominal angle given above. For example, if the datasheet
                             // for a servo says that 1000us is the minimum signal correspond
                             // to an angle of 0.0 degrees, but testing shows that at 1000us, the servo is
                             // actually at 10.0 degrees, then set this to 10.0. The library will then
                             // attempt to interpolate down to 0.0 degrees by adjusting the actual
                             // PWM.

    // PWM Parameters
    float min_angle_pwm;        // PWM pulse duration (in us) for the minimum angle
    float max_angle_pwm;        // PWM pulse duration (in us) for the maximum angle
    float pwm_timer_increment;  // PWM pulse duration increment (in us) for each additional timer step
                                // (1 / freq of timer peripheral). (Usually set by the devicetree.)
    float pwm_per_degree;       // us of pulse width per degree, signed; negative for an inverted calibration.
                                // Derived from the four calibration fields; 0 when the span cannot give a usable
                                // slope. Never assign directly — see `motor_math.c`.

    // Servo-specific internal state
    float last_time_generated;      // In seconds
    float last_position_generated;  // In degrees
    float known_position;
} servo_motor_context_t;

typedef struct stepper_motor_context {
    motor_motion_profile_t motion_profile;
    // Stepper-specific parameters
    // STEP pin parameters
    float min_step;              // 1 / microstep; Usually 1, 0.5, 0.25, 0.125, etc.
    float timer_increment;       // inverse of the frequency of the timer peripheral used for the stepper
    float steps_per_revolution;  // Number of steps per 360 degree/2pi radian revolution of the stepper motor

    // Stepper-specific internal state
    float last_time_generated;      // In seconds
    float last_position_generated;  // In revolutions
} stepper_motor_context_t;

/**
 * Whether a single angle is one a servo may be configured or commanded to: finite and within
 * `0..SERVO_MAX_ALLOWED_ANGLE` degrees.
 */
bool motor_motion_servo_angle_valid(float angle);

/**
 * Whether a single pulse duration is one the timer can produce: finite, non-negative and shorter than the PWM
 * period (`SERVO_MAX_PULSE_DURATION_US`).
 */
bool motor_motion_servo_pwm_duration_valid(float duration_us);

/**
 * Whether both angle limits are individually valid. This is a range check on each field, not an ordering
 * check: an inverted pair (`min > max`) and a degenerate pair (`min == max`) are both valid here.
 */
bool motor_motion_servo_angles_valid(float min_angle, float max_angle);

/**
 * Whether both calibration pulse durations are individually valid.
 */
bool motor_motion_servo_pwm_durations_valid(float min_angle_pwm, float max_angle_pwm);

/**
 * Convert an (actual) angle in degrees to the timer count that produces the corresponding pulse, using the
 * servo's two-point calibration. Saturates at `0` and `UINT32_MAX` rather than performing an undefined cast.
 */
uint32_t motor_motion_servo_degrees_to_pwm_count(const servo_motor_context_t *context, float degree);

/**
 * Initializes the context struct with the parameters from the paper.
 *
 * @param start start position of the motor. See note in the struct definition for units.
 * @param end end position of the motor. See note in the struct definition for units.
 * @param max_velocity maximum velocity of the motor. See note in the struct definition for units.
 * @param max_acceleration maximum acceleration of the motor. See note in the struct definition for units.
 * @param min_angle_pwm the pulse duration in microseconds for the minimum angle.
 * @param max_angle_pwm the pulse duration in microseconds for the maximum angles.
 *
 * @retval 0 on success
 * @retval -errno on error
 */
int motor_motion_servo_init_context_struct(float start, float end, float max_velocity, float max_acceleration,
                                           float min_angle_pwm, float max_angle_pwm, servo_motor_context_t *context);

/**
 * Initialize the context struct with the parameters from the paper.
 *
 * @param start start position of the motor. See note in the struct definition for units.
 * @param end end position of the motor. See note in the struct definition for units.
 * @param max_velocity maximum velocity of the motor. See note in the struct definition for units.
 * @param max_acceleration maximum acceleration of the motor. See note in the struct definition for units.
 * @param microsteps Number of micro steps per step
 * @param steps_per_revolution the number of steps for an entire revolution of the motor.
 * @param timer_increment is the inverse of the frequency of the timer peripheral used for the stepper.
 *
 * @retval 0 on success
 * @retval -errno on error
 */
int motor_motion_stepper_init_context_struct(float start, float end, float max_velocity, float max_acceleration,
                                             uint16_t microsteps, float timer_increment, float steps_per_revolution,
                                             stepper_motor_context_t *context);

/**
 * Generates a table of servo displacements for a sinusoidal motion profile uses a fixed increment of 0.02 seconds
 * but will never exceed the size of the table given.
 *
 * Runs based on the `servo_last_time_generated` field in the context struct, to allow for multiple
 * calls (for instance, one can send a pre-generated table to the servo driver, then call this function on
 * a different buffer to generate the next table). See note in the struct definition for units.
 *
 * @return the number of entries generated or -1 on error.
 */
ssize_t motor_motion_servo_generate_displacement_table(uint32_t *table, size_t table_size,
                                                       servo_motor_context_t *context);

/**
 * Generates a table of values with a pulse at the correct time for each stamp.
 *
 * @return the number of entries generated or -1 on error.
 */
ssize_t motor_motion_stepper_generate_timing_table(uint32_t *table, size_t table_size,
                                                   stepper_motor_context_t *context);
