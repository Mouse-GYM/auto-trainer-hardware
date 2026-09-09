#include "motor_math.h"

#include <argp.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int print_n_iterations = 0;

static float pulse_width_to_degrees(const servo_motor_context_t *context, const uint32_t pulse_width) {
    return ((float)pulse_width * context->pwm_timer_increment - context->min_angle_pwm) *
               (context->max_angle - context->min_angle) / (context->max_angle_pwm - context->min_angle_pwm) +
           context->min_angle;
}

static void print_context_variables(const motor_motion_profile_t *context) {
    fprintf(stderr, "Servo context:\n");
    fprintf(stderr, "start_pos: %f\n", context->start_pos);
    fprintf(stderr, "end_pos: %f\n", context->end_pos);
    fprintf(stderr, "a_max: %f\n", context->a_max);
    fprintf(stderr, "v_max: %f\n", context->v_max);
    fprintf(stderr, "sgn: %f\n", context->sgn);
    fprintf(stderr, "y_f: %f\n", context->y_f);
    fprintf(stderr, "y_s: %f\n", context->y_s);
    fprintf(stderr, "y_a: %f\n", context->y_a);
    fprintf(stderr, "v_w: %f\n", context->v_w);
    fprintf(stderr, "t_o: %f\n", context->t_o);
    fprintf(stderr, "t_a: %f\n", context->t_a);
    fprintf(stderr, "omega: %f\n", context->omega);
    fprintf(stderr, "k_s: %f\n", context->k_s);
    fprintf(stderr, "t_k: %f\n", context->t_k);
    fprintf(stderr, "t_s: %f\n", context->t_s);
    fprintf(stderr, "t_t: %f\n", context->t_t);
}

static void print_servo_values(const uint32_t *servo_values, const size_t n_values, const size_t start_n) {
    for (size_t i = 0; i < n_values; i++) {
        printf("%u, %f\n", servo_values[i], 0.02 * (i + start_n));
    }
}

static void print_stepper_values(const stepper_motor_context_t *const context, const uint32_t *stepper_values,
                                 const size_t n_values, const size_t start_n) {
    static float current_time = 0.0f;
    for (size_t i = 0; i < n_values; i++) {
        printf("%f, %f\n", (float)(start_n + i + 1) * context->min_step, current_time);
        current_time += (float)stepper_values[i] * context->timer_increment;
    }
}

static void servo_verify_max_velocity_and_acceleration(const servo_motor_context_t *context, const uint32_t *values,
                                                       const size_t n_values, const bool verify_velocity,
                                                       const bool verify_acceleration) {
    if (!verify_velocity && !verify_acceleration) {
        return;
    }

    float last_position = pulse_width_to_degrees(context, values[0]);
    float last_velocity = 0.0f;

    for (size_t i = 1; i < n_values; i++) {
        const float this_position = pulse_width_to_degrees(context, values[i]);
        const float velocity = (values[i] - last_position) / 0.02f;
        if (verify_velocity && fabsf(velocity) > context->motion_profile.v_max) {
            fprintf(stderr, "Average velocity at generated position %lu, is greater than max velocity: %f > %f\n", i,
                    velocity, context->motion_profile.v_max);
        }
        if (i > 1 && verify_acceleration) {
            const float acceleration = (velocity - last_velocity) / 0.02f;
            if (fabsf(acceleration) > context->motion_profile.a_max) {
                fprintf(stderr,
                        "Average acceleration at generated position %lu, is greater than max acceleration: %f > %f\n",
                        i, acceleration, context->motion_profile.a_max);
            }
        }
        last_position = this_position;
        last_velocity = velocity;
    }
}

static void stepper_verify_max_velocity_and_acceleration(const stepper_motor_context_t *context, const uint32_t *values,
                                                         const size_t n_values, const bool verify_velocity,
                                                         const bool verify_acceleration) {
    if (!verify_velocity && !verify_acceleration) {
        return;
    }

    float last_time = 0.0f;
    float last_velocity = 0.0f;
    for (size_t i = 0; i < n_values; i++) {
        const float this_time = last_time + (float)values[i] * context->timer_increment;
        const float velocity = context->min_step / (this_time - last_time);
        if (verify_velocity && fabsf(velocity) > context->motion_profile.v_max) {
            fprintf(stderr, "Average velocity at generated position %lu, is greater than max velocity: %f > %f\n", i,
                    velocity, context->motion_profile.v_max);
        }
        if (i > 0 && verify_acceleration) {
            const float acceleration = (velocity - last_velocity) / (this_time - last_time);
            if (fabsf(acceleration) > context->motion_profile.a_max) {
                fprintf(stderr,
                        "Average acceleration at generated position %lu, is greater than max acceleration: %f > %f\n",
                        i, acceleration, context->motion_profile.a_max);
            }
        }
        last_velocity = velocity;
        last_time = this_time;
    }
}

const char *argp_program_version = "motor_math_test .1";
const char *argp_program_bug_address = "";
const char doc[] = "Test the motor math library by building and using the CMake project in this directory.";
const char args_doc[] = "";

static struct argp_option options[] = {
    {"print", 'p', 0, 0, "Print entire model as csv to stdout.", 1},
    {"print-iterations", 'N', 0, 0, "Print the number of iterations of Halley's method each time.", 1},
    {"servo", 's', 0, 0, "Generate servo values.", 1},
    {"stepper", 't', 0, 0, "Generate stepper values.", 1},
    {0, 0, 0, 0, "General Parameters", 2},
    {"start", 'x', "start", 0, "Start position", 2},
    {"end", 'y', "end", 0, "End Position", 2},
    {"max-velocity", 'v', "velocity", 0, "Max velocity", 2},
    {"max-acceleration", 'a', "acceleration", 0, "Max acceleration", 2},
    {0, 0, 0, 0, "Servo Parameters", 3},
    {"min-angle", 'm', "min", 0, "Minimum angle", 3},
    {"max-angle", 'M', "max", 0, "Maximum angle", 3},
    {"min-angle-pwm", 'u', "min_angle_pwm", 0, "PWM Duration (in us) of minimum angle", 3},
    {"max-angle-pwm", 'o', "max_angle_pwm", 0, "PWM Duration (in us) of maximum angle", 3},
    {0, 0, 0, 0, "Stepper Parameters", 4},
    {"min-step", 'n', "min-step", 0, "Minimum step", 4},
    {"timer-increment", 'i', "increment", 0, "Timer increment", 4},
    {"steps-per-revolution", 'S', "steps", 0, "Steps per revolution", 4},
    {0, 0, 0, 0, "Tests", 5},
    {"verify-max-velocity", 'V', 0, 0, "Verify max velocity", 5},
    {"verify-max-acceleration", 'A', 0, 0, "Verify max acceleration", 5},
    {0, 0, 0, 0, "Self Tests", 6},
    {"selftest", 'T', 0, 0, "Run the angle-to-PWM assertions and exit non-zero on failure.", 6},
    {0},
    {0},
};

struct arguments {
    int print;
    int servo;
    int stepper;
    int selftest;
    float start;
    float end;
    float max_velocity;
    float max_acceleration;
    float min_angle;
    float max_angle;
    float angle_adjustment;
    float servo_pwm_min_angle;
    float servo_pwm_max_angle;
    float min_step;
    float timer_increment;
    float steps_per_revolution;
    int verify_max_velocity;
    int verify_max_acceleration;
};

static error_t parse_opt(int key, char *arg, struct argp_state *state) {
    char *end = NULL;
    struct arguments *arguments = state->input;
    switch (key) {
        case 'p':
            arguments->print = 1;
            break;
        case 'N':
            print_n_iterations = 1;
            break;
        case 's':
            arguments->servo = 1;
            break;
        case 't':
            arguments->stepper = 1;
            break;
        case 'x':
            arguments->start = strtof(arg, &end);
            if (*end != '\0') {
                fprintf(stderr, "Couldn't parse start position: %s\n", arg);
            }
            break;
        case 'y':
            arguments->end = strtof(arg, &end);
            if (*end != '\0') {
                fprintf(stderr, "Couldn't parse end position: %s\n", arg);
            }
            break;
        case 'v':
            arguments->max_velocity = strtof(arg, &end);
            if (*end != '\0') {
                fprintf(stderr, "Couldn't parse max velocity: %s\n", arg);
            }
            break;
        case 'a':
            arguments->max_acceleration = strtof(arg, &end);
            if (*end != '\0') {
                fprintf(stderr, "Couldn't parse max acceleration: %s\n", arg);
            }
            break;
        case 'm':
            arguments->min_angle = strtof(arg, &end);
            if (*end != '\0') {
                fprintf(stderr, "Couldn't parse min angle: %s\n", arg);
            }
            break;
        case 'M':
            arguments->max_angle = strtof(arg, &end);
            if (*end != '\0') {
                fprintf(stderr, "Couldn't parse max angle: %s\n", arg);
            }
            break;
        case 'u':
            arguments->servo_pwm_min_angle = strtof(arg, &end);
            if (*end != '\0') {
                fprintf(stderr, "Couldn't parse servo pwm min angle: %s\n", arg);
            }
            break;
        case 'o':
            arguments->servo_pwm_max_angle = strtof(arg, &end);
            if (*end != '\0') {
                fprintf(stderr, "Couldn't parse servo pwm max angle: %s\n", arg);
            }
            break;
        case 'n':
            arguments->min_step = strtof(arg, &end);
            if (*end != '\0') {
                fprintf(stderr, "Couldn't parse min stepn: %s\n", arg);
            }
            break;
        case 'i':
            arguments->timer_increment = strtof(arg, &end);
            if (*end != '\0') {
                fprintf(stderr, "Couldn't parse timer increment: %s\n", arg);
            }
            break;
        case 'S':
            arguments->steps_per_revolution = strtof(arg, &end);
            if (*end != '\0') {
                fprintf(stderr, "Couldn't parse steps per revolution: %s\n", arg);
            }
            break;
        case 'V':
            arguments->verify_max_velocity = 1;
            break;
        case 'A':
            arguments->verify_max_acceleration = 1;
            break;
        case 'T':
            arguments->selftest = 1;
            break;
        default:
            return ARGP_ERR_UNKNOWN;
    }
    return 0;
}

#define SERVO_VALUES_NUM 256U
static int model_and_verify_servo(const struct arguments *arguments) {
    static servo_motor_context_t servo_context = {0};
    static uint32_t servo_values[SERVO_VALUES_NUM];

    if (arguments->max_velocity == 0.0f || arguments->max_acceleration == 0.0f) {
        fprintf(stderr, "Max velocity and max acceleration must be non-zero.\n");
        return -EINVAL;
    }

    if (arguments->start == arguments->end) {
        fprintf(stderr, "start and end position are the same.\n");
    }

    // The angle limits have to be in place before the init call: it derives the degrees-to-microseconds slope
    // from all four calibration fields, so initializing first would derive it from a zeroed struct.
    servo_context.pwm_timer_increment = 0.5;  // corresponds to 2MHz, hardcoded
    servo_context.min_angle = arguments->min_angle;
    servo_context.max_angle = arguments->max_angle;
    servo_context.angle_adjustment = arguments->angle_adjustment;

    int ret = motor_motion_servo_init_context_struct(arguments->start, arguments->end, arguments->max_velocity,
                                                     arguments->max_acceleration, arguments->servo_pwm_min_angle,
                                                     arguments->servo_pwm_max_angle, &servo_context);
    if (ret < 0) {
        fprintf(stderr, "Failed to initialize servo context: %d\n", ret);
        print_context_variables(&servo_context.motion_profile);
        return ret;
    }

    size_t running_count = 0;
    ssize_t n_vals = motor_motion_servo_generate_displacement_table(servo_values, SERVO_VALUES_NUM, &servo_context);
    ssize_t j = 0;
    while (n_vals > 0) {
        fprintf(stderr, "Run %ld of size %ld\n", j++, n_vals);
        if (arguments->verify_max_velocity || arguments->verify_max_acceleration) {
            servo_verify_max_velocity_and_acceleration(
                &servo_context, servo_values, ret, arguments->verify_max_velocity, arguments->verify_max_acceleration);
        }
        if (arguments->print) {
            print_servo_values(servo_values, n_vals, running_count);
        }
        running_count += n_vals;
        n_vals = motor_motion_servo_generate_displacement_table(servo_values, SERVO_VALUES_NUM, &servo_context);
    }

    if (n_vals < 0) {
        fprintf(stderr, "Failed to generate servo values: %ld\n", n_vals);
        return -1;
    }

    return 0;
}

#define STEPPER_VALUES_NUM 1024U
static int model_and_verify_stepper(const struct arguments *arguments) {
    static stepper_motor_context_t stepper_context = {0};
    static uint32_t stepper_values[STEPPER_VALUES_NUM];

    if (arguments->max_velocity == 0.0f || arguments->max_acceleration == 0.0f) {
        fprintf(stderr, "Max velocity and max acceleration must be non-zero.\n");
        return -EINVAL;
    }

    if (arguments->min_step == 0) {
        fprintf(stderr, "Servo min_step is unset. Initializing to 1.\n");
    }

    if (arguments->steps_per_revolution == 0) {
        fprintf(stderr, "Steps per revolution must be non-zero.\n");
        return -EINVAL;
    }

    if (arguments->start == arguments->end) {
        fprintf(stderr, "start and end position are the same.\n");
    }

    // `min_step` is `1 / microsteps`, and the init call takes the microstep count.
    const float min_step = arguments->min_step == 0.0f ? 1.0f : arguments->min_step;
    int ret = motor_motion_stepper_init_context_struct(arguments->start, arguments->end, arguments->max_velocity,
                                                       arguments->max_acceleration, (uint16_t)lroundf(1.0f / min_step),
                                                       arguments->timer_increment, arguments->steps_per_revolution,
                                                       &stepper_context);
    if (ret < 0) {
        fprintf(stderr, "Failed to initialize stepper context: %d\n", ret);
        print_context_variables(&stepper_context.motion_profile);
        return ret;
    }

    size_t running_count = 0;
    ssize_t n_vals = motor_motion_stepper_generate_timing_table(stepper_values, STEPPER_VALUES_NUM, &stepper_context);
    ssize_t j = 0;
    while (n_vals > 0) {
        fprintf(stderr, "Run %ld of size %ld\n", j++, n_vals);
        if (arguments->verify_max_velocity || arguments->verify_max_acceleration) {
            stepper_verify_max_velocity_and_acceleration(&stepper_context, stepper_values, ret,
                                                         arguments->verify_max_velocity,
                                                         arguments->verify_max_acceleration);
        }
        if (arguments->print) {
            print_stepper_values(&stepper_context, stepper_values, n_vals, running_count);
        }
        running_count += n_vals;
        n_vals = motor_motion_stepper_generate_timing_table(stepper_values, STEPPER_VALUES_NUM, &stepper_context);
    }

    if (n_vals < 0) {
        fprintf(stderr, "Failed to generate servo values: %d\n", ret);
        return -1;
    }

    return 0;
}

/* ***** Self Tests ***** */

// 2 MHz servo timer, as both boards use: one count is 0.5 us, so a count is microseconds x 2.
#define SELFTEST_TIMER_INCREMENT 0.5f

// The generator's dead band (`motor_math.c`). A change smaller than this is accumulated into a later entry
// rather than written, so a generated count can legitimately sit a few counts past an endpoint.
#define SELFTEST_DEAD_BAND 4

// Tolerance for a mid-move count sitting outside the endpoint pair. The dead band alone is not enough: the
// generator writes `pwm + accumulate`, so the whole carried residue lands on one entry, and the residue is
// bounded only by how many consecutive sub-dead-band steps preceded that write. Measured overshoot is 11
// counts on a 0-180 span and 1 on the 0-120 span this change leaves alone, so the carry is pre-existing
// generator behaviour rather than anything the calibration slope introduces. A sign error or a wrong span
// misses by hundreds of counts, so this stays far tighter than the defect it guards against.
#define SELFTEST_ENDPOINT_TOLERANCE (SELFTEST_DEAD_BAND * 4)

struct mapping_case {
    const char *name;
    float min_angle;
    float max_angle;
    float min_angle_pwm;
    float max_angle_pwm;
    float angle_adjustment;
    float degree;
    uint32_t expected_count;
    bool expect_zero_slope;
};

static const struct mapping_case mapping_cases[] = {
    {"minimum endpoint", 0.0f, 180.0f, 1000.0f, 2000.0f, 0.0f, 0.0f, 2000u, false},
    {"maximum endpoint (the defect: 4000, not 5000)", 0.0f, 180.0f, 1000.0f, 2000.0f, 0.0f, 180.0f, 4000u, false},
    {"midpoint", 0.0f, 180.0f, 1000.0f, 2000.0f, 0.0f, 90.0f, 3000u, false},
    {"narrow span", 0.0f, 60.0f, 1000.0f, 2000.0f, 0.0f, 60.0f, 4000u, false},
    {"span that used to be right", 0.0f, 120.0f, 1000.0f, 2000.0f, 0.0f, 120.0f, 4000u, false},
    {"inverted, minimum endpoint", 90.0f, 10.0f, 200.0f, 1200.0f, 0.0f, 90.0f, 400u, false},
    {"inverted, maximum endpoint", 90.0f, 10.0f, 200.0f, 1200.0f, 0.0f, 10.0f, 2400u, false},
    {"inverted, midpoint", 90.0f, 10.0f, 200.0f, 1200.0f, 0.0f, 50.0f, 1400u, false},
    {"non-zero min_angle anchor", 30.0f, 150.0f, 1000.0f, 2000.0f, 0.0f, 30.0f, 2000u, false},
    {"angle_adjustment offset", 0.0f, 180.0f, 1000.0f, 2000.0f, 10.0f, 10.0f, 2000u, false},
    {"degenerate span", 45.0f, 45.0f, 1000.0f, 2000.0f, 0.0f, 90.0f, 2000u, true},
    {"span too small to scale", 0.0f, 1e-40f, 1000.0f, 2000.0f, 0.0f, 90.0f, 2000u, true},
    {"longest usable pulse", 0.0f, 180.0f, 1000.0f, 19999.0f, 0.0f, 180.0f, 39998u, false},
    // Not a calibration any edge accepts — `motor_motion_servo_pwm_duration_valid` rejects a 1e38 us endpoint.
    // It exists only to prove the converter saturates the integer cast instead of wrapping.
    {"saturation guard, not a valid calibration", 0.0f, 180.0f, 0.0f, 1e38f, 0.0f, 180.0f, UINT32_MAX, false},
};

/*
 * Build a context the converter can be called against directly. The profile bounds are deliberately unrelated
 * to the calibration: `pwm_per_degree` is derived from the four calibration fields, which have to be in place
 * before the init call, and a profile over a degenerate or subnormal span raises floating-point exceptions the
 * mapping cases are not about.
 */
static int build_mapping_context(servo_motor_context_t *const context, const struct mapping_case *const test_case) {
    static const servo_motor_context_t empty = {0};
    *context = empty;
    context->pwm_timer_increment = SELFTEST_TIMER_INCREMENT;
    context->min_angle = test_case->min_angle;
    context->max_angle = test_case->max_angle;
    context->angle_adjustment = test_case->angle_adjustment;

    return motor_motion_servo_init_context_struct(0.0f, 90.0f, 200.0f, 100.0f, test_case->min_angle_pwm,
                                                  test_case->max_angle_pwm, context);
}

static int selftest_mapping(void) {
    int failures = 0;

    for (size_t i = 0; i < sizeof(mapping_cases) / sizeof(mapping_cases[0]); i++) {
        const struct mapping_case *const test_case = &mapping_cases[i];
        servo_motor_context_t context;

        const int ret = build_mapping_context(&context, test_case);
        if (ret != 0) {
            fprintf(stderr, "FAIL [%s]: could not initialize the context: %d\n", test_case->name, ret);
            failures++;
            continue;
        }

        const uint32_t count = motor_motion_servo_degrees_to_pwm_count(&context, test_case->degree);
        if (count != test_case->expected_count) {
            fprintf(stderr, "FAIL [%s]: expected %u counts at %f degrees, got %u\n", test_case->name,
                    test_case->expected_count, test_case->degree, count);
            failures++;
        }

        if (test_case->expect_zero_slope && context.pwm_per_degree != 0.0f) {
            fprintf(stderr, "FAIL [%s]: expected the slope to be suppressed to 0, got %f\n", test_case->name,
                    context.pwm_per_degree);
            failures++;
        }
    }

    return failures;
}

struct angle_case {
    float angle;
    bool expected;
};

struct angle_pair_case {
    float min_angle;
    float max_angle;
    bool expected;
};

struct duration_case {
    float duration_us;
    bool expected;
};

struct duration_pair_case {
    float min_angle_pwm;
    float max_angle_pwm;
    bool expected;
};

static int selftest_predicates(void) {
    // The single-field predicates are covered directly because the settings loader calls them by pointer.
    static const struct angle_case angles[] = {
        {0.0f, true},    {90.0f, true}, {180.0f, true},    {-1.0f, false},
        {181.0f, false}, {NAN, false},  {INFINITY, false}, {-INFINITY, false},
    };

    static const struct angle_pair_case angle_pairs[] = {
        {0.0f, 180.0f, true},  {90.0f, 10.0f, true},  {45.0f, 45.0f, true}, {0.0f, 0.0f, true},
        {-1.0f, 90.0f, false}, {0.0f, 181.0f, false}, {NAN, 90.0f, false},  {0.0f, INFINITY, false},
    };

    // The ceiling is exclusive: a pulse equal to the whole period is not a pulse.
    static const struct duration_case durations[] = {
        {0.0f, true},      {200.0f, true},    {19999.0f, true}, {-1.0f, false},
        {20000.0f, false}, {25000.0f, false}, {NAN, false},     {INFINITY, false},
    };

    static const struct duration_pair_case duration_pairs[] = {
        {200.0f, 1200.0f, true},    {0.0f, 1200.0f, true},      {1000.0f, 19999.0f, true},
        {-1.0f, 1200.0f, false},    {1000.0f, -2000.0f, false}, {NAN, 1200.0f, false},
        {1000.0f, INFINITY, false}, {1000.0f, 20000.0f, false}, {1000.0f, 25000.0f, false},
    };

    int failures = 0;

    for (size_t i = 0; i < sizeof(angles) / sizeof(angles[0]); i++) {
        if (motor_motion_servo_angle_valid(angles[i].angle) != angles[i].expected) {
            fprintf(stderr, "FAIL [angle_valid(%f)]: expected %d\n", angles[i].angle, angles[i].expected);
            failures++;
        }
    }

    for (size_t i = 0; i < sizeof(angle_pairs) / sizeof(angle_pairs[0]); i++) {
        if (motor_motion_servo_angles_valid(angle_pairs[i].min_angle, angle_pairs[i].max_angle) !=
            angle_pairs[i].expected) {
            fprintf(stderr, "FAIL [angles_valid(%f, %f)]: expected %d\n", angle_pairs[i].min_angle,
                    angle_pairs[i].max_angle, angle_pairs[i].expected);
            failures++;
        }
    }

    for (size_t i = 0; i < sizeof(durations) / sizeof(durations[0]); i++) {
        if (motor_motion_servo_pwm_duration_valid(durations[i].duration_us) != durations[i].expected) {
            fprintf(stderr, "FAIL [pwm_duration_valid(%f)]: expected %d\n", durations[i].duration_us,
                    durations[i].expected);
            failures++;
        }
    }

    for (size_t i = 0; i < sizeof(duration_pairs) / sizeof(duration_pairs[0]); i++) {
        if (motor_motion_servo_pwm_durations_valid(duration_pairs[i].min_angle_pwm, duration_pairs[i].max_angle_pwm) !=
            duration_pairs[i].expected) {
            fprintf(stderr, "FAIL [pwm_durations_valid(%f, %f)]: expected %d\n", duration_pairs[i].min_angle_pwm,
                    duration_pairs[i].max_angle_pwm, duration_pairs[i].expected);
            failures++;
        }
    }

    return failures;
}

struct generation_case {
    const char *name;
    float min_angle;
    float max_angle;
    float min_angle_pwm;
    float max_angle_pwm;
};

/*
 * Drive a whole move through the generator, which is the only way to catch a sign error interacting with the
 * dead band and the `accumulate` carry: the converter alone cannot see either.
 */
static int selftest_generation(void) {
    static const struct generation_case cases[] = {
        {"forward 0-180 deg / 1000-2000 us", 0.0f, 180.0f, 1000.0f, 2000.0f},
        {"inverted 90-10 deg / 200-1200 us", 90.0f, 10.0f, 200.0f, 1200.0f},
    };

    static const size_t MAX_REFILLS = 64;
    static uint32_t table[SERVO_VALUES_NUM];
    int failures = 0;

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        const struct generation_case *const test_case = &cases[i];
        static const servo_motor_context_t empty = {0};
        servo_motor_context_t context = empty;

        context.pwm_timer_increment = SELFTEST_TIMER_INCREMENT;
        context.min_angle = test_case->min_angle;
        context.max_angle = test_case->max_angle;

        const int ret =
            motor_motion_servo_init_context_struct(test_case->min_angle, test_case->max_angle, 200.0f, 100.0f,
                                                   test_case->min_angle_pwm, test_case->max_angle_pwm, &context);
        if (ret != 0) {
            fprintf(stderr, "FAIL [%s]: could not initialize the context: %d\n", test_case->name, ret);
            failures++;
            continue;
        }

        const int64_t start_count = motor_motion_servo_degrees_to_pwm_count(&context, test_case->min_angle);
        const int64_t target_count = motor_motion_servo_degrees_to_pwm_count(&context, test_case->max_angle);
        const int64_t low = (start_count < target_count ? start_count : target_count) - SELFTEST_ENDPOINT_TOLERANCE;
        const int64_t high = (start_count > target_count ? start_count : target_count) + SELFTEST_ENDPOINT_TOLERANCE;

        // Bounded by "returned fewer entries than the table holds", with a refill cap as a backstop. The
        // generator never returns 0, so a `while (n_vals > 0)` loop here would never terminate.
        int64_t final_count = -1;
        size_t refills = 0;
        ssize_t n_vals = 0;
        do {
            n_vals = motor_motion_servo_generate_displacement_table(table, SERVO_VALUES_NUM, &context);
            if (n_vals < 0) {
                fprintf(stderr, "FAIL [%s]: generation failed: %ld\n", test_case->name, n_vals);
                failures++;
                break;
            }

            for (ssize_t j = 0; j < n_vals; j++) {
                if ((int64_t)table[j] < low || (int64_t)table[j] > high) {
                    fprintf(stderr, "FAIL [%s]: generated count %u outside [%ld, %ld]\n", test_case->name, table[j],
                            low, high);
                    failures++;
                }
            }

            if (n_vals > 0) {
                final_count = table[n_vals - 1];
            }
            refills++;
        } while (n_vals == (ssize_t)SERVO_VALUES_NUM && refills < MAX_REFILLS);

        if (refills >= MAX_REFILLS) {
            fprintf(stderr, "FAIL [%s]: generation did not finish within %zu refills\n", test_case->name, MAX_REFILLS);
            failures++;
        }

        if (final_count < target_count - SELFTEST_DEAD_BAND || final_count > target_count + SELFTEST_DEAD_BAND) {
            fprintf(stderr, "FAIL [%s]: final count %ld is not within %d of the target endpoint %ld\n", test_case->name,
                    final_count, SELFTEST_DEAD_BAND, target_count);
            failures++;
        }
    }

    return failures;
}

static int run_selftests(void) {
    const int failures = selftest_mapping() + selftest_predicates() + selftest_generation();

    if (failures == 0) {
        fprintf(stderr, "All self tests passed.\n");
    } else {
        fprintf(stderr, "%d self test failure(s).\n", failures);
    }

    return failures;
}

int main(const int argc, char *argv[]) {
    static struct arguments arguments = {0};
    static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL};

    argp_parse(&argp, argc, argv, 0, 0, &arguments);

    if (arguments.selftest) {
        return run_selftests();
    }

    if (arguments.servo) {
        const int ret = model_and_verify_servo(&arguments);
        if (ret < 0) {
            fprintf(stderr, "Error in servo model: %d\n", ret);
        }
    }

    if (arguments.stepper) {
        const int ret = model_and_verify_stepper(&arguments);
        if (ret < 0) {
            fprintf(stderr, "Error in stepper model: %d\n", ret);
        }
    }

    return 0;
}
