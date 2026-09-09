#include <stdbool.h>
#include <stdlib.h>
#include <zephyr/shell/shell.h>

#include "motor_motion.h"

LOG_MODULE_REGISTER(motor_math, LOG_LEVEL_DBG);

static int cmd_servo_set_pwm_parameters(const struct shell *shell, size_t argc, char **argv) {
    if (argc != 4) {
        shell_print(shell, "Invalid number of arguments");
        return -EINVAL;
    }

    char *endptr;
    const int servo = strtol(argv[1], &endptr, 10);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse servo number");
        return -EINVAL;
    }

    const float pwm_min_angle = strtof(argv[2], &endptr);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse pwm min angle");
        return -EINVAL;
    }

    const float pwm_max_angle = strtof(argv[3], &endptr);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse pwm max angle");
        return -EINVAL;
    }

    const struct device *servo_dev = servo_motor_by_id(servo);
    if (servo_dev == NULL) {
        shell_print(shell, "Invalid servo number");
        return -EINVAL;
    }

    const int ret = servo_set_parameters(servo_dev, 0.0f, 0.0f, pwm_min_angle, pwm_max_angle);
    if (ret != 0) {
        shell_print(shell, "Failed to set servo parameters: %d", ret);
        return -EINVAL;
    }

    return 0;
}

static int cmd_servo_set_angle_parameters(const struct shell *shell, const size_t argc, char **argv) {
    if (argc != 4) {
        shell_print(shell, "Invalid number of arguments");
        return -EINVAL;
    }

    char *end;
    const int servo = strtol(argv[1], &end, 10);
    if (*end != '\0') {
        shell_print(shell, "Couldn't parse servo number");
        return -EINVAL;
    }

    const float min_angle = strtof(argv[2], &end);
    if (*end != '\0') {
        shell_print(shell, "Couldn't parse min angle");
        return -EINVAL;
    }

    const float max_angle = strtof(argv[3], &end);
    if (*end != '\0') {
        shell_print(shell, "Couldn't parse max angle");
        return -EINVAL;
    }

    const struct device *servo_dev = servo_motor_by_id(servo);
    if (servo_dev == NULL) {
        shell_print(shell, "Invalid servo number");
        return -EINVAL;
    }

    const int ret = servo_set_angle_parameters(servo_dev, min_angle, max_angle);
    if (ret != 0) {
        shell_print(shell, "Couldn't set angle parameters: %d", ret);
        return ret;
    }

    return 0;
}

static int cmd_servo_set_physical_parameters(const struct shell *shell, size_t argc, char **argv) {
    if (argc != 4) {
        shell_print(shell, "Invalid number of arguments");
        return -EINVAL;
    }

    char *endptr;
    const int servo = strtol(argv[1], &endptr, 10);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse servo number");
        return -EINVAL;
    }

    const float max_velocity = strtof(argv[2], &endptr);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse max velocity");
        return -EINVAL;
    }

    const float max_acceleration = strtof(argv[3], &endptr);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse max acceleration");
        return -EINVAL;
    }

    const struct device *servo_dev = servo_motor_by_id(servo);
    if (servo_dev == NULL) {
        shell_print(shell, "Invalid servo number");
        return -EINVAL;
    }

    const int ret = servo_set_parameters(servo_dev, max_velocity, max_acceleration, -1, -1);
    if (ret != 0) {
        shell_print(shell, "Failed to set servo parameters: %d", ret);
    }

    return 0;
}

static int cmd_servo_move(const struct shell *shell, size_t argc, char **argv) {
    if (argc != 3) {
        shell_print(shell, "Invalid number of arguments");
        return -EINVAL;
    }

    char *endptr;
    const int servo = strtol(argv[1], &endptr, 10);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse servo number");
        return -EINVAL;
    }

    const struct device *servo_dev = servo_motor_by_id(servo);
    if (servo_dev == NULL) {
        shell_print(shell, "Invalid servo number");
        return -EINVAL;
    }

    const float position = strtof(argv[2], &endptr);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse position");
        return -EINVAL;
    }

    const int ret = servo_move_to_position(servo_dev, position, 200, 100);

    if (ret != 0) {
        shell_print(shell, "Failed to move servo to position: %d", ret);
        return -EINVAL;
    }

    return 0;
}

static int cmd_servo_stop(const struct shell *shell, size_t argc, char **argv) {
    if (argc != 2) {
        shell_print(shell, "Invalid number of arguments");
        return -EINVAL;
    }

    char *endptr;
    const int servo = strtol(argv[1], &endptr, 10);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse stepper number");
        return -EINVAL;
    }

    const struct device *servo_dev = servo_motor_by_id(servo);
    if (servo_dev == NULL) {
        shell_print(shell, "Invalid servo number");
        return -EINVAL;
    }

    servo_motor_stop(servo_dev);
    servo_cancel_all_work(servo_dev);
    return 0;
}

static int cmd_stepper_set_steps(const struct shell *shell, size_t argc, char **argv) {
    if (argc != 5) {
        shell_print(shell, "Invalid number of arguments");
        return -EINVAL;
    }

    char *endptr;
    const int stepper = strtol(argv[1], &endptr, 10);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse stepper number");
        return -EINVAL;
    }

    const float min_step = strtof(argv[2], &endptr);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse min step");
        return -EINVAL;
    }

    const float steps_per_revolution = strtof(argv[3], &endptr);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse steps per revolution");
        return -EINVAL;
    }

    const struct device *const stepper_dev = stepper_motor_by_id(stepper);
    if (stepper_dev == NULL) {
        shell_print(shell, "Invalid stepper number");
        return -EINVAL;
    }

    const int flip_limit_orientation = strtol(argv[4], &endptr, 10);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse flip_limit_orientation");
    }

    const int ret =
        stepper_set_parameters(stepper_dev, 0.0f, 0.0f, 0.0f, min_step, steps_per_revolution, flip_limit_orientation);

    if (ret != 0) {
        shell_print(shell, "Failed to set stepper parameters: %d", ret);
        return -EINVAL;
    }
    return 0;
}

static int cmd_stepper_set_physical_parameters(const struct shell *shell, size_t argc, char **argv) {
    if (argc != 5) {
        shell_print(shell, "Invalid number of arguments");
        return -EINVAL;
    }
    char *endptr;
    const int stepper = strtol(argv[1], &endptr, 10);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse stepper number");
        return -EINVAL;
    }

    const float max_velocity = strtof(argv[2], &endptr);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse max velocity");
        return -EINVAL;
    }

    const float max_acceleration = strtof(argv[3], &endptr);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse max acceleration");
        return -EINVAL;
    }

    const struct device *const stepper_dev = stepper_motor_by_id(stepper);
    if (stepper_dev == NULL) {
        shell_print(shell, "Invalid stepper number");
        return -EINVAL;
    }

    const int flip_limit_orientation = strtol(argv[4], &endptr, 10);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse flip_limit_orientation");
    }

    const int ret =
        stepper_set_parameters(stepper_dev, max_velocity, max_acceleration, 0.0f, 0, 0.0f, flip_limit_orientation);
    if (ret != 0) {
        shell_print(shell, "Failed to set stepper parameters: %d", ret);
        return -EINVAL;
    }
    return 0;
}

static int cmd_stepper_move(const struct shell *shell, size_t argc, char **argv) {
    if (argc != 5) {
        shell_print(shell, "Invalid number of arguments");
        return -EINVAL;
    }
    char *endptr;
    const int stepper = strtol(argv[1], &endptr, 10);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse stepper number");
        return -EINVAL;
    }

    const float position = strtof(argv[2], &endptr);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse position");
        return -EINVAL;
    }

    const float max_v = strtof(argv[3], &endptr);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse max acceleration");
        return -EINVAL;
    }

    const float max_a = strtof(argv[4], &endptr);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse max velocity");
        return -EINVAL;
    }

    const struct device *const stepper_dev = stepper_motor_by_id(stepper);
    if (stepper_dev == NULL) {
        shell_print(shell, "Invalid stepper number");
        return -EINVAL;
    }

    const int ret = stepper_move_to_position(stepper_dev, position, max_v, max_a);
    if (ret != 0) {
        shell_print(shell, "Failed to move stepper to position: %d", ret);
        return -EINVAL;
    }
    return 0;
}

static int cmd_stepper_stop(const struct shell *shell, size_t argc, char **argv) {
    if (argc != 2) {
        shell_print(shell, "Invalid number of arguments");
        return -EINVAL;
    }

    char *endptr;
    const int stepper = strtol(argv[1], &endptr, 10);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse stepper number");
        return -EINVAL;
    }

    const struct device *const stepper_dev = stepper_motor_by_id(stepper);
    if (stepper_dev == NULL) {
        shell_print(shell, "Invalid stepper number");
        return -EINVAL;
    }

    stepper_motor_stop(stepper_dev);
    return 0;
}

static int cmd_stepper_home(const struct shell *shell, size_t argc, char **argv) {
    if (argc != 2) {
        shell_print(shell, "Invalid number of arguments");
        return -EINVAL;
    }

    char *endptr;
    const int stepper = strtol(argv[1], &endptr, 10);
    if (*endptr != '\0') {
        shell_print(shell, "Couldn't parse stepper number");
        return -EINVAL;
    }

    const struct device *dev = stepper_motor_by_id(stepper);
    if (dev == NULL) {
        shell_print(shell, "Invalid stepper number");
        return -EINVAL;
    }

    const int ret = stepper_home(dev);
    if (ret != 0) {
        shell_print(shell, "Failed to home slowly");
        return -EIO;
    }

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_motor_math,
    SHELL_CMD_ARG(servo_set_pwm, NULL,
                  "Set the PWM parameters (duration in us for the minimum and maximum angle) for a servo.\nUsage: "
                  "servo_set_pwm <servo> "
                  "<pwm_min_angle_us> <pwm_max_angle_us>",
                  cmd_servo_set_pwm_parameters, 4, 0),
    SHELL_CMD_ARG(
        servo_set_angles, NULL,
        "Set the minimum and maximum angles for a servo.\nUsage: servo_set_angles <servo> <min_angle> <max_angle>",
        cmd_servo_set_angle_parameters, 4, 0),
    SHELL_CMD_ARG(servo_set_physical, NULL,
                  "Set the Physical parameters (max velocity and acceleration) for a servo.\nUsage: servo_set_physical "
                  "<servo> <max_velocity> <max_acceleration>",
                  cmd_servo_set_physical_parameters, 4, 0),
    SHELL_CMD_ARG(servo_move, NULL, "Set a servo position sinusoidally\nUsage: servo_move <servo> <position>",
                  cmd_servo_move, 3, 0),
    SHELL_CMD_ARG(servo_stop, NULL, "Stop a servo\nUsage: servo_stop <servo>", cmd_servo_stop, 2, 0),
    SHELL_CMD_ARG(
        stepper_set_steps, NULL,
        "Set the steps per revolution and minimum step for a stepper motor.\nUsage: stepper_set_steps <stepper> "
        "<min_step> <steps_per_revolution> <flip_limit_orientation>",
        cmd_stepper_set_steps, 5, 0),
    SHELL_CMD_ARG(stepper_set_physical, NULL,
                  "Set the Physical parameters (max velocity and acceleration) for a stepper motor.\nUsage: "
                  "stepper_set_physical <stepper> <max_velocity> <max_acceleration> <flip_limit_orientation>",
                  cmd_stepper_set_physical_parameters, 5, 0),
    SHELL_CMD_ARG(stepper_move, NULL,
                  "Move a stepper motor to the specified position sinusoidally.\nUsage: stepper_move <stepper> "
                  "<position> <movement_max_velocity> <movement_max_acceleration>",
                  cmd_stepper_move, 5, 0),
    SHELL_CMD_ARG(stepper_stop, NULL, "Stop a stepper motor\nUsage: stepper_stop <stepper>", cmd_stepper_stop, 2, 0),
    SHELL_CMD_ARG(stepper_home, NULL, "Home a stepper\nUsage: stepper_home <stepper>", cmd_stepper_home, 2, 0),
    SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(motor_math, &sub_motor_math, "Motor math motion commands", NULL);
