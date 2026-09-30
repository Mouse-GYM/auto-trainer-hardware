#pragma once

#include <stdatomic.h>
#include <zephyr/device.h>
#include <zephyr/sys/atomic.h>

#include "motor_callbacks.h"
#include "motor_math.h"
#include "servo.h"
#include "stepper.h"

/* ***** Struct Declarations and Defaults ***** */
#define SERVO_BUFFER_SIZE 256
#define STEPPER_BUFFER_SIZE 1024
#define BUFS_PER_MOTOR 2

typedef uint32_t servo_buffer_set[BUFS_PER_MOTOR][SERVO_BUFFER_SIZE];
typedef uint32_t stepper_buffer_set[BUFS_PER_MOTOR][STEPPER_BUFFER_SIZE];

// MOTION_FAULT: a move was aborted: a stepper's by the protection monitor, a failed homing verification or the
// e-stop; a servo's by the e-stop. The CAN layer acks it with an error (`stepper_motion_error` for a stepper,
// -ECANCELED for a servo) and returns it to MOTION_IDLE, as it does for MOTION_DONE.
typedef enum { MOTION_IDLE, MOTION_IN_PROGESS, MOTION_DONE, MOTION_FAULT } motion_mode_t;

// Stepper protection faults (`stepper_work_context.fault`). Latched until `stepper_clear_fault`.
#define STEPPER_FAULT_OVERTEMP_WARNING BIT(0)  // Driver reported otpw during a move.
#define STEPPER_FAULT_OVERTEMP BIT(1)          // Driver reported ot during a move.
// A move or homing didn't finish within its timeout, or homing covered its maximum travel without reaching the
// limit switch.
#define STEPPER_FAULT_MOVE_TIMEOUT BIT(2)
#define STEPPER_FAULT_DRIVER_COMM BIT(3)  // DRV_STATUS couldn't be read during a move.

// How far homing moves away from the limit switch to verify it, in move position units (mm on the pellet module).
#define STEPPER_HOMING_VERIFY_DISTANCE 1.0f

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

// Once homing has found the limit switch, it proves the switch works: a normal move away from it, a check that it
// released, and a normal move back to 0. Each value names the step `homing_verify_work` takes next.
typedef enum {
    HOMING_VERIFY_NONE,    // Not verifying.
    HOMING_VERIFY_START,   // At the switch, zeroed; the move away is next.
    HOMING_VERIFY_AWAY,    // Moving away; the switch must be released when it ends.
    HOMING_VERIFY_RETURN,  // Moving back to 0; reaching the switch on the way re-zeroes there.
} homing_verify_t;

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
    bool homing_ramp_done;  // Homing has finished accelerating and now steps at the homing velocity.
    // Pulses homing may still generate before it has covered CONFIG_LIB_MOTOR_MOTION_STEPPER_HOMING_MAX_TRAVEL.
    uint32_t homing_pulses_left;
    // Set from the DMA callback when homing played out all its pulses without reaching the limit switch; the
    // monitor turns it into a fault.
    bool homing_travel_exhausted;
    _Atomic homing_verify_t homing_verify;
    // Why a move or homing ended in MOTION_FAULT when no `fault` bit explains it: the homing verification's error,
    // or -ECANCELED for the e-stop. 0 otherwise.
    int homing_error;
    struct k_work_delayable calculation_work;
    struct k_work_delayable check_driver_work;
    struct k_work_delayable homing_verify_work;
    ll_stepper_cb_t stepper_cb;

    // Protection. While `fault` has any STEPPER_FAULT_* bit set, moves and homing are refused. `driver_disabled`
    // means the driver's power stage was confirmed off (toff = 0). `move_deadline_ms` is the uptime by which the
    // current move or homing must finish, 0 for none.
    atomic_t fault;
    bool driver_disabled;
    int64_t move_deadline_ms;
    uint8_t drv_status_read_failures;

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
 * @retval -ENODEV if the device is not found among the static context structs.
 */
int servo_set_parameters(const struct device *dev, float max_velocity, float max_acceleration, float min_angle_pwm,
                         float max_angle_pwm);

/*
 * Set the minimum and maximum angles of a servo. Both must be set at the same time.
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
 * it is unchanged. Nothing changes unless the whole configuration is accepted.
 *
 * @retval -ENODEV if the device is not found among the static context structs.
 * @retval -EBUSY if the motor is moving.
 * @retval -EINVAL for a non-finite value, or microsteps that aren't a power of two from 1 to 256.
 * @retval -errno if the driver didn't take the microstep setting.
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
 * Move to the position specified, using the motion profiles in `motor_math.*`. While the move runs, the driver's
 * DRV_STATUS is polled; on otpw or ot, or if the move outlasts its timeout (proportional to the distance), the
 * move is aborted, the driver is disabled, and `motion_mode` becomes MOTION_FAULT.
 *
 * The velocity is capped at what the step timer can produce at the current microstep setting.
 *
 * @retval -ENODEV if the device is not found in the list.
 * @retval -EBUSY if another motion profile is already running, or the motor must be homed after an e-stop or a
 *         failed homing.
 * @retval -ECANCELED if the e-stop is engaged.
 * @retval -EPERM if a protection fault is latched; see `stepper_clear_fault`.
 * @retval -EAGAIN if the target is less than one step pulse away.
 */
int stepper_move_to_position(const struct device *dev, float target_position, float max_velocity,
                             float max_acceleration);

/*
 * Move relative to the current position. Wrapper around `stepper_move_to_position`.
 */
int stepper_move_relative(const struct device *dev, float delta_position, float max_velocity, float max_acceleration);

/*
 * Drive toward the limit switch, ramping up to the homing velocity. DRV_STATUS is polled as for a move. Homing
 * stops stepping once it has covered CONFIG_LIB_MOTOR_MOTION_STEPPER_HOMING_MAX_TRAVEL and faults with
 * STEPPER_FAULT_MOVE_TIMEOUT if the switch wasn't reached, or sooner if it outlasts a move's timeout for that
 * distance. Returns -EPERM if a protection fault is latched.
 *
 * At the switch (or if it's already active), the position becomes 0 and the switch is verified: a normal move of
 * STEPPER_HOMING_VERIFY_DISTANCE away from it, which must release it, then a normal move back to 0, re-zeroing
 * wherever the switch closes again. A switch that stays active still gets the move back, then ends homing in
 * MOTION_FAULT with `homing_error` -ENXIO, and moves are refused with -EBUSY until homing succeeds.
 */
int stepper_home(const struct device *dev);

/**
 * Clear a latched protection fault and re-enable the driver. Blocks for the driver's UART exchanges (tens of
 * ms); don't call from an ISR. Clearing does nothing about the position: the aborted move stopped short of
 * `last_position_generated`, so the caller should home before relying on it.
 *
 * @retval 0 if no fault is latched, or once the driver reads back as enabled and the fault is cleared.
 * @retval -ENODEV if the device is not found in the list.
 * @retval -EBUSY if the motor is moving.
 * @retval -EAGAIN if the driver still reports otpw or ot; the fault stays latched.
 * @retval -EIO or another -errno if the driver can't be read or re-enabled; the fault stays latched.
 */
int stepper_clear_fault(const struct device *dev);

/**
 * @return the latched STEPPER_FAULT_* bits, 0 if none or the device is not found. Safe to call from ISRs.
 */
uint32_t stepper_get_fault(const struct device *dev);

/**
 * @return whether the protection has confirmed the driver's power stage is off. Safe to call from ISRs.
 */
bool stepper_driver_output_disabled(const struct device *dev);

/**
 * @return the error to acknowledge a move aborted with `fault`: -ETIMEDOUT for a move timeout alone,
 *         otherwise -EIO.
 */
int stepper_fault_error(uint32_t fault);

/**
 * @return the error to acknowledge a stepper move or homing that ended in MOTION_FAULT: the homing verification's
 *         error if that is what failed, otherwise `stepper_fault_error` of the latched fault.
 */
int stepper_motion_error(const struct device *dev);

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
 * Whether the servo's angle limits can be reasoned against at all: both finite and correctly ordered. Nothing
 * validates them on either install path — the settings loader passes `+Inf` through and the CAN cfg setter
 * checks nothing — so every reader that compares against or clamps to them has to ask first.
 */
bool servo_angle_limits_usable(const struct servo_work_context *context);

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
 * E-stop one stepper: stop the pulses, drop queued blocks and cancel refills and homing verification. A move or
 * homing in progress ends in MOTION_FAULT with -ECANCELED, and the motor must be homed before it moves again.
 * Blocks until running work items finish; don't call from an ISR.
 */
void stepper_e_stop(const struct device *dev);

/**
 * E-stop one servo: stop the pulse-width updates and drop queued blocks, so the horn holds where it is. A move in
 * progress ends in MOTION_FAULT. Blocks until a running refill finishes; don't call from an ISR.
 */
void servo_e_stop(const struct device *dev);

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
