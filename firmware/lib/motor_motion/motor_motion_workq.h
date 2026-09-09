#pragma once

#include <stdatomic.h>
#include <zephyr/device.h>

#include "motor_callbacks.h"
#include "motor_math.h"
#include "servo.h"
#include "stepper.h"

/* ***** Struct Declarations and Defaults ***** */
// Default pwm duration of the minimum angle
#define SERVO_DEFAULT_MIN_ANGLE_PWM 1000.0f
// Default pwm duration of the maximum angle
#define SERVO_DEFAULT_MAX_ANGLE_PWM 2000.0f
// Default minimum angle of servo
#define SERVO_DEFAULT_MIN_ANGLE 0.0f
// Default maximum angle of servo
#define SERVO_DEFAULT_MAX_ANGLE SERVO_MAX_ALLOWED_ANGLE

#define SERVO_BUFFER_SIZE 256
#define STEPPER_BUFFER_SIZE 1024
#define BUFS_PER_MOTOR 2

typedef uint32_t servo_buffer_set[BUFS_PER_MOTOR][SERVO_BUFFER_SIZE];
typedef uint32_t stepper_buffer_set[BUFS_PER_MOTOR][STEPPER_BUFFER_SIZE];

typedef enum { MOTION_IDLE, MOTION_IN_PROGESS, MOTION_DONE } motion_mode_t;

struct servo_work_context {
    const struct device *dev;  // Motor device to use

    // Parameters for the current movement
    servo_motor_context_t context;
    servo_buffer_set buffers;
    size_t current_buffer;
    ssize_t last_calculation_ret;

    // Parameters about movement overall
    atomic_flag e_stop_triggered;
    motion_mode_t motion_mode;
    bool motion_calculation_done;

    // While `position_assumed` is set, `known_position` and `last_position_generated` hold `min_angle` and
    // follow it; an accepted stored position or a started move ends that permanently. `position_valid` means
    // firmware drove the horn to `known_position` this power cycle — a restored value does not qualify.
    // `pending_position` stages the stored record between load and commit, and `persisted_position` is the
    // only value the settings export may write. Both are NAN when there is nothing there.
    bool position_assumed;
    bool position_valid;
    float pending_position;
    float persisted_position;

    struct k_work_delayable calculation_work;
    struct k_work_delayable submission_work;
    ll_servo_cb_t servo_cb;
    float motor_max_velocity;
    float motor_max_acceleration;
    uint8_t uuid;
};

typedef enum { MOVING_HOME, MOVING_POSITION } movement_control_t;

struct stepper_work_context {
    const struct device *dev;  // Motor device to use

    // Parameters for the current movement
    stepper_motor_context_t context;
    stepper_buffer_set buffers;
    size_t current_buffer;
    ssize_t last_calculation_ret;

    // Parameters about overall movement state
    atomic_flag e_stop_triggered;
    _Atomic movement_control_t move_control;
    _Atomic ll_stepper_dir_t motor_direction;
    motion_mode_t motion_mode;
    bool motion_calculation_done;
    struct k_work_delayable calculation_work;
    struct k_work_delayable check_driver_work;
    ll_stepper_cb_t stepper_cb;

    // Motion parameters that should be constant for the motor
    // (Load these from the settings, they are rarely changed.)
    float motor_max_velocity;
    float motor_max_acceleration;
    float homing_velocity;
    float motor_steps_per_revolution;
    float fixed_position;
    uint16_t microsteps;  // micro steps per step; should be a power of 2.
    bool flip_limit_orientation;

    // Command-based information
    float timer_increment;  // Time (in seconds) between successive pulses of the timer.
    uint8_t uuid;
};

/**
 * These parameters are more constant than the positions so abstracted out here. If the `float`
 * parameters are lower than or equal to 0.0f, then those parameters are unchanged.
 *
 * A non-finite argument is rejected outright — neither `NaN` nor `-Inf` is a usable value or a legitimate way
 * to say "unchanged" — as is a positive pulse duration the timer cannot produce (see
 * `SERVO_MAX_PULSE_DURATION_US`).
 *
 * @retval -ENODEV if the device is not found among the static context structs.
 * @retval -EINVAL if either pulse duration is non-finite or is a positive value out of range.
 */
int servo_set_parameters(const struct device *dev, float max_velocity, float max_acceleration, float min_angle_pwm,
                         float max_angle_pwm);

/*
 * Set the minimum and maximum angles of a servo. Both must be set at the same time. The two are calibration
 * endpoints rather than ordered bounds, so an inverted pair (`min > max`) is accepted; each must be finite and
 * within `0..SERVO_MAX_ALLOWED_ANGLE` degrees, and a rejected pair leaves both fields at their previous values.
 *
 * @retval -ENODEV if the device is not found among the static context structs.
 * @retval -EINVAL if either angle is out of range.
 */
int servo_set_angle_parameters(const struct device *dev, const float min_angle, const float max_angle);

/**
 * Move to the position specified, using the motion profiles in `motor_math.*`.
 *
 * @retval -ENODEV if the device is not found in the list.
 * @retval -EBUSY if another motion profile is already running.
 */
int servo_move_to_position(const struct device *dev, float target_position, float max_velocity, float max_acceleration);

/*
 * Move relative to the current position. Wrapper around `servo_move_to_position`.
 */
int servo_move_relative(const struct device *dev, float delta_position, float max_velocity, float max_acceleration);

/**
 * These parameters are usually constant across movements of the motor, so we abstract them to a
 * separate function. If any of these parameters have values lower than or equal to 0.0f,
 * it is unchanged.
 *
 * @retval -ENODEV if the device is not found among the static context structs.
 */
int stepper_set_parameters(const struct device *dev, float motor_max_velocity, float motor_max_acceleration,
                           float homing_velocity, uint16_t microsteps, float motor_steps_per_revolution,
                           bool flip_limit_orientation);

/**
 * Save an X, Y, or Z position as part of the 'send' capability.
 *
 * @param context - context
 * @param motor_id - motor ID [0..2] where 0=X, 1=Y, 2=Z
 * @param position
 * @return 0 on success; <0 on failure
 */
int stepper_save_fixed_location(struct stepper_work_context *context, int motor_id, float position, bool is_absolute);

/**
 * Move to the position specified, using the motion profiles in `motor_math.*`.
 *
 * @retval -ENODEV if the device is not found in the list.
 * @retval -EBUSY if another motion profile is already running.
 */
int stepper_move_to_position(const struct device *dev, float target_position, float max_velocity,
                             float max_acceleration);

/*
 * Move relative to the current position. Wrapper around `stepper_move_to_position`.
 */
int stepper_move_relative(const struct device *dev, float delta_position, float max_velocity, float max_acceleration);

int stepper_home(const struct device *dev);

/*
 * Cancel all work on the motor.
 */
void stepper_cancel_all_work(const struct device *dev);

/*
 * Cancel all work on the motor.
 */
void servo_cancel_all_work(const struct device *dev);

/*
 * Zero out the internal state of this library, as after hitting a limit switch. (Presumably after the
 * motors have been stopped.) This does not move the motor, just sets the internal "zero point" of this
 * library.
 */
void stepper_set_position_to_zero(const struct device *dev);
void servo_set_position_to_zero(const struct device *dev);

/*
 * Adopt `min_angle` as the believed position of a servo that has never had a real one. A no-op once a stored
 * position has been accepted or a move has started, so changing the angle limits cannot move the believed
 * position of a servo whose horn firmware has already driven.
 */
void servo_assume_min_angle_position(struct servo_work_context *context);

/*
 * Whether the servo's angle limits can be reasoned against at all: the two are calibration endpoints in either
 * order, so this asks only whether both are finite and they describe a non-degenerate span. A stored record
 * predating the load-path validation can still carry a non-finite or degenerate pair, so every reader that
 * compares against or clamps to them has to ask first.
 */
bool servo_angle_limits_usable(const struct servo_work_context *context);

/*
 * Whether `position` lies within the servo's configured travel, whichever of the two limits is the larger.
 * False when the limits are not usable at all.
 */
bool servo_position_within_limits(const struct servo_work_context *context, float position);

/*
 * Find the work contexts, given the device.
 */
struct servo_work_context *find_servo_context_from_device(const struct device *dev);

/*
 * Find the work contexts, given the device.
 */
struct stepper_work_context *find_stepper_context_from_device(const struct device *dev);

struct stepper_work_context *stepper_work_context_from_motor_id(int motor_id);

/**
 * Get the minimum angle of the servo motor.
 *
 * @returns 0 on success, -ENODEV if the device is not found.
 */

typedef struct {
    float min_position;
    float max_position;
    float min_pwm_duration_us;
    float max_pwm_duration_us;
    float motor_max_velocity;
    float motor_max_acceleration;
} servo_config_t;

int servo_read_config(const struct device *dev, servo_config_t *config);

/**
 * Internally invoked by the library during an e-stop so that no motion happens
 * unless the homing procedure is followed.
 */
void set_all_e_stop_flags(void);

/**
 * @param dev stepper device
 * @return The current homing status of the motor
 */
movement_control_t stepper_homing_status(const struct device *dev);

typedef struct stepper_config {
    bool flip_limit_orientation;
    float steps_per_revolution;
    float motor_max_velocity;
    float motor_max_acceleration;
    float homing_velocity;
    uint16_t microsteps;
} stepper_config_t;

int stepper_read_config(const struct device *dev, struct stepper_config *config);
