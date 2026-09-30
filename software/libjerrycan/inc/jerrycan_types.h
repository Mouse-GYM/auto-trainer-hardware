#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(__ZEPHYR__)
#include <linux/can.h>

#include <cassert>
#define BUILD_ASSERT(cond, msg) static_assert(cond, msg)
typedef void* sys_snode_t;
#define JERRYCAN_ACTUAL_PAYLOAD_SIZE (CANFD_MAX_DLEN)
#else
#include <zephyr/drivers/can.h>
#define JERRYCAN_ACTUAL_PAYLOAD_SIZE (CAN_MAX_DLEN)
#endif

typedef uint8_t uuid_t;

#define JERRYCAN_MAX_PAYLOAD_SIZE (JERRYCAN_ACTUAL_PAYLOAD_SIZE - sizeof(uuid_t))  // account for UUID

#define SIZE_CHECK(type, expected_size) \
    BUILD_ASSERT(sizeof(type) == (expected_size), #type " should be " #expected_size " bytes")

typedef enum __attribute__((packed)) {
    JERRYCAN_CMD_ESTOP = 0x00,
    JERRYCAN_CMD_HEARTBEAT = 0x3F,
    JERRYCAN_CMD_STATUS = 0x01,
    JERRYCAN_CMD_STEPPER_MOVE = 0x02,
    JERRYCAN_CMD_SERVO_MOVE = 0x03,
    JERRYCAN_CMD_STEPPER_HOME = 0x04,
    JERRYCAN_CMD_CFG_WRITE = 0x05,
    JERRYCAN_CMD_CFG_READ = 0x06,
    JERRYCAN_CMD_CFG_RESPONSE = 0x07,
    JERRYCAN_CMD_PRESSURE_READ = 0x08,
    JERRYCAN_CMD_TEMP_HUM_READ = 0x09,
    JERRYCAN_CMD_GPIO_READ = 0x0A,
    JERRYCAN_CMD_GPIO_WRITE = 0x0B,
    JERRYCAN_CMD_TONE = 0X0C,
    JERRYCAN_CMD_ANALOG_OUT = 0X0D,
    JERRYCAN_CMD_LOAD_CELL_READ = 0x0E,
    JERRYCAN_CMD_DOOR_SENSOR = 0x0F,
    JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_BEGIN = 0x10,
    JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_CONT = 0x11,
    JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_END = 0x12,
    JERRYCAN_CMD_RGB_LED = 0x13,
    JERRYCAN_CMD_LOAD_CELL_TARE = 0x14,
    JERRYCAN_RSVD1 = 0x15,
    JERRYCAN_CMD_STEPPER_STATUS = 0x16,
    JERRYCAN_CMD_SERVO_STATUS = 0x17,
    JERRYCAN_CMD_BOOTLOADER_COMMAND = 0x18,
    JERRYCAN_CMD_BOOTLOADER_RESPONSE = 0x19,
    JERRYCAN_CMD_BOOTLOADER_DATA = 0x1A,
    JERRYCAN_CMD_DELAY = 0x1B,
    JERRYCAN_CMD_FIXED_XYZ = 0x1C,
    JERRYCAN_CMD_SERVO_ATTACH = 0x1D,
    JERRYCAN_CMD_SERVO_DETACH = 0x1E,
    JERRYCAN_CMD_STEPPER_FAULT_CLEAR = 0x1F,
    JERRYCAN_RSP_ACK = 0x30,
    JERRYCAN_CMD_MIN = 0x00,
    JERRYCAN_CMD_MAX = 0x3F,
} jerrycan_cmd_type_t;

typedef struct __attribute__((packed)) {
    uint8_t rsvd;  // Nonzero engages the e-stop, 0 releases it.
} jerrycan_cmd_estop_t;

SIZE_CHECK(jerrycan_cmd_estop_t, 1);

typedef struct __attribute__((packed)) {
    uint8_t rsvd;
} jerrycan_cmd_heartbeat_t;

SIZE_CHECK(jerrycan_cmd_heartbeat_t, 1);

typedef struct __attribute__((packed)) {
    struct {
        uint8_t estop_active : 1;
        uint8_t limit_switch0 : 1;
        uint8_t limit_switch1 : 1;
        uint8_t limit_switch2 : 1;
        uint8_t button0 : 1;
        uint8_t rsvd0 : 3;
    };
    uint8_t rsvd1;
    uint8_t stepper_status0;
    uint8_t stepper_status1;
    uint8_t stepper_status2;
    uint8_t servo_status0;
    uint8_t servo_status1;
    uint8_t servo_status2;
} jerrycan_cmd_status_t;

SIZE_CHECK(jerrycan_cmd_status_t, 8);

typedef enum __attribute__((packed)) {
    JERRYCAN_MOVE_ABSOLUTE = 0,
    JERRYCAN_MOVE_RELATIVE = 1,
} abs_or_rel_t;

typedef struct __attribute__((packed)) {
    struct {
        uint8_t motor_id : 2;
        uint8_t save : 1;
        uint8_t rsvd0 : 4;
        abs_or_rel_t abs_or_rel : 1;
    };
    float position;
    float max_velocity;
    float max_acceleration;
} jerrycan_cmd_stepper_move_t;

SIZE_CHECK(jerrycan_cmd_stepper_move_t, 13);

// Home a stepper on its limit switch, then verify the switch with a normal move 1 unit (1 mm on the pellet module)
// away from it, which must release it, and a normal move back to 0. Acked when that is done; the status reports
// homing_status 0 throughout. -ENXIO, after the move back, if the switch stayed active (defective or stuck):
// further moves are refused with -EBUSY until homing succeeds.
typedef struct __attribute__((packed)) {
    uint8_t motor_id : 7;
    uint8_t rsvd : 1;
} jerrycan_cmd_stepper_home_t;

SIZE_CHECK(jerrycan_cmd_stepper_home_t, 1);

// Clear a stepper's latched protection fault (see jerrycan_stepper_status_flags_t) and re-enable its driver.
// Acked with 0 when the driver is enabled again or nothing was latched; -EAGAIN while the driver still reports
// otpw or ot; -EBUSY while the motor is moving; -EIO if the driver can't be reached. The aborted move stopped
// short of the reported position, so home before relying on it.
typedef struct __attribute__((packed)) {
    uint8_t motor_id;
} jerrycan_cmd_stepper_fault_clear_t;

SIZE_CHECK(jerrycan_cmd_stepper_fault_clear_t, 1);

// I think the payload for this message can be the same format as the stepper move message
typedef jerrycan_cmd_stepper_move_t jerrycan_cmd_servo_move_t;


typedef struct __attribute__((packed)) {
    uint8_t motor_id;
} jerrycan_cmd_servo_attach_t;

typedef jerrycan_cmd_servo_attach_t jerrycan_cmd_servo_detach_t;

typedef enum __attribute__((packed)) {
    JERRYCAN_CFG_STEPPER,
    JERRYCAN_CFG_SERVO,
} jerrycan_cfg_type_t;

typedef struct __attribute__((packed)) {
    struct __attribute__((packed)) {
        uint8_t motor_id : 2;
        uint8_t error : 1;
        uint8_t rsvd0 : 5;
    };
    int16_t rsvd;
    float min_position;
    float max_position;
    float min_pwm_duration_us;
    float max_pwm_duration_us;
    float motor_max_velocity;
    float motor_max_acceleration;
} jerrycan_servo_cfg_t;

typedef struct __attribute__((packed)) {
    struct __attribute__((packed)) {
        uint8_t motor_id : 2;
        uint8_t error : 1;
        bool flip_limit_orientation : 1;
        uint8_t rsvd0 : 4;
    };
    uint16_t microsteps;  // Power of 2; number of micro steps in a step
    float steps_per_revolution;
    float motor_max_velocity;
    float motor_max_acceleration;
    float homing_velocity;
} jerrycan_stepper_cfg_t;

typedef struct __attribute__((packed)) {
    jerrycan_cfg_type_t type;
    union {
        jerrycan_servo_cfg_t servo;
        jerrycan_stepper_cfg_t stepper;
    };
} jerrycan_cmd_cfg_t;

SIZE_CHECK(jerrycan_cmd_cfg_t, 28);

typedef struct __attribute__((packed)) {
    uint8_t instance;
    uint32_t pressure;
} jerrycan_cmd_pressure_read_t;

SIZE_CHECK(jerrycan_cmd_pressure_read_t, 5);

/*
    Scale factor for temperature and humidity transmission
    (provides 2 significant decimal digits of precision)
*/
#define TEMPHUM_SCALE_FACTOR 100

typedef struct __attribute__((packed)) {
    uint8_t instance;
    /*
        SI7021 Temperature Range -40C to +125C - Sensor data temperature field is a float.
        This field will contain (uint16_t)(temperature * 100) to provided two decimal
        places of precision without bloating the struct beyond the 8 byte limit. As such, the
        recieving end must perform (float)(temperature / 100.0) to acquire the temperature value.
    */
    uint16_t temperature;
    /*
        SI7021 Humidity Range 0% to 100% - Sensor data humidity field is a float.
        This field will contain (uint16_t)(humidity * 100) to provided two decimal
        places of precision without bloating the struct beyond the 8 byte limit. As such, the
        recieving end must perform (float)(humidity / 100.0) to acquire the humidity value.
    */
    uint16_t humidity;
} jerrycan_cmd_temp_hum_read_t;

SIZE_CHECK(jerrycan_cmd_temp_hum_read_t, 5);

typedef struct __attribute__((packed)) {
    uint8_t instance;
    uint32_t state;
} jerrycan_cmd_gpio_read_t;

SIZE_CHECK(jerrycan_cmd_gpio_read_t, 5);

typedef struct __attribute__((packed)) {
    uint8_t instance;
    uint16_t gpio_idx;
    bool state : 1;
    uint8_t rsvd0 : 7;
} jerrycan_cmd_gpio_write_t;

SIZE_CHECK(jerrycan_cmd_gpio_write_t, 4);

typedef struct __attribute__((packed)) {
    uint8_t instance;
    uint16_t frequency_hz;
    uint16_t duration_ms;
} jerrycan_cmd_tone_t;

SIZE_CHECK(jerrycan_cmd_tone_t, 5);

typedef struct __attribute__((packed)) {
    uint8_t instance;
    uint16_t value_mv;
} jerrycan_cmd_analog_out_t;

SIZE_CHECK(jerrycan_cmd_analog_out_t, 3);

typedef struct __attribute__((packed)) {
    uint8_t instance;
    float load_mv;
} jerrycan_cmd_load_cell_read_t;

SIZE_CHECK(jerrycan_cmd_load_cell_read_t, 5);

typedef struct __attribute__((packed)) {
    uint8_t door1 : 1;  // bit flags: opened(1), closed(0)
    uint8_t door2 : 1;
    uint8_t door3 : 1;
    uint8_t external_button : 1;
} jerrycan_cmd_door_closed_t;

SIZE_CHECK(jerrycan_cmd_door_closed_t, 1);

typedef struct __attribute__((packed)) {
    uint8_t red;    // (%)
    uint8_t green;  // (%)
    uint8_t blue;   // (%)
} jerrycan_cmd_rgb_led_t;

SIZE_CHECK(jerrycan_cmd_rgb_led_t, 3);

typedef struct __attribute__((packed)) {
    uint32_t stream_id;
} jerrycan_cmd_audio_data_cmd_t;

SIZE_CHECK(jerrycan_cmd_audio_data_cmd_t, 4);

typedef struct {
    union {
        uint8_t payload[JERRYCAN_ACTUAL_PAYLOAD_SIZE];
        float magnitudes[JERRYCAN_ACTUAL_PAYLOAD_SIZE / sizeof(float)];
    };
} jerrycan_cmd_audio_data_t;

SIZE_CHECK(jerrycan_cmd_audio_data_t, 64);

typedef struct __attribute__((packed)) {
    uint8_t instance;
} jerrycan_cmd_load_cell_tare_t;

SIZE_CHECK(jerrycan_cmd_load_cell_tare_t, 1);

typedef struct __attribute__((packed)) {
    uint8_t motor_id;
    uint8_t status;
    uint8_t homing_status;
    uint8_t limit_switch;
    float position;
    float send_position;
} jerrycan_cmd_stepper_status_t;

SIZE_CHECK(jerrycan_cmd_stepper_status_t, 12);

// Bits of `jerrycan_cmd_stepper_status_t.status`; 0 means no fault.
// 0x01-0x04 come from the stepper driver's configuration check at boot and are held until reset.
// 0x08-0x40 are protection faults latched during a move or homing. The move is aborted and acked with -EIO, or
// -ETIMEDOUT for MOVE_TIMEOUT alone; further moves and homing are refused with -EPERM until
// JERRYCAN_CMD_STEPPER_FAULT_CLEAR succeeds.
// 0x80 reports that the protection switched the driver's power stage off; the motor freewheels.
typedef enum __attribute__((packed)) {
    JERRYCAN_STEPPER_STATUS_DRIVER_UART_FAULT = 0x01,     // Driver didn't answer a verification read.
    JERRYCAN_STEPPER_STATUS_DRIVER_WRITE_LOST = 0x02,     // Driver didn't accept every configuration write.
    JERRYCAN_STEPPER_STATUS_DRIVER_READBACK_FAULT = 0x04, // Driver register didn't read back as written.
    JERRYCAN_STEPPER_STATUS_OVERTEMP_WARNING = 0x08,      // Driver reported otpw (temperature prewarning).
    JERRYCAN_STEPPER_STATUS_OVERTEMP = 0x10,              // Driver reported ot (overtemperature shutdown).
    JERRYCAN_STEPPER_STATUS_MOVE_TIMEOUT = 0x20,          // Move or homing timed out, or homing missed the switch.
    JERRYCAN_STEPPER_STATUS_DRIVER_COMM_LOST = 0x40,      // DRV_STATUS couldn't be read during the move.
    JERRYCAN_STEPPER_STATUS_DRIVER_DISABLED = 0x80,       // Driver power stage off (CHOPCONF.toff = 0).
} jerrycan_stepper_status_flags_t;

typedef struct __attribute__((packed)) {
    uint8_t motor_id;
    uint8_t status;
    float position;
} jerrycan_cmd_servo_status_t;

SIZE_CHECK(jerrycan_cmd_servo_status_t, 6);

typedef enum __attribute__((packed)) {
    JERRYCAN_BOOTLOADER_SUBCMD_VERSION,
    JERRYCAN_BOOTLOADER_SUBCMD_START,
    JERRYCAN_BOOTLOADER_SUBCMD_END,
    JERRYCAN_BOOTLOADER_SUBCMD_REBOOT,
    JERRYCAN_BOOTLOADER_SUBCMD_FINALIZE,
    JERRYCAN_BOOTLOADER_SUBCMD_ACK,
    JERRYCAN_BOOTLOADER_SUBCMD_NACK,
} jerrycan_bootloader_subcmd_t;

typedef struct __attribute__((packed)) {
    jerrycan_bootloader_subcmd_t type;
} jerrycan_cmd_bootloader_command_t;

typedef struct __attribute__((packed)) {
    uint8_t running_version_major;
    uint8_t running_version_minor;
    uint8_t running_version_patch;
    uint8_t slot1_version_major;
    uint8_t slot1_version_minor;
    uint8_t slot1_version_patch;
} jerrycan_cmd_bootloader_version_t;

SIZE_CHECK(jerrycan_cmd_bootloader_version_t, 6);

typedef struct __attribute__((packed)) {
    uint8_t active : 1;
    uint8_t : 7;
    uint32_t bytes_written;
} jerrycan_cmd_bootloader_status_t;

SIZE_CHECK(jerrycan_cmd_bootloader_status_t, 5);

typedef struct __attribute__((packed)) {
    jerrycan_bootloader_subcmd_t type;
    union {
        jerrycan_cmd_bootloader_version_t version;
        jerrycan_cmd_bootloader_status_t status;
    };
} jerrycan_cmd_bootloader_response_t;

SIZE_CHECK(jerrycan_cmd_bootloader_response_t, 7);

typedef struct {
    uint8_t data[JERRYCAN_ACTUAL_PAYLOAD_SIZE];
} jerrycan_cmd_bootloader_data_t;

SIZE_CHECK(jerrycan_cmd_bootloader_data_t, 64);

typedef struct __attribute__((packed)) {
    uint16_t delay;
} jerrycan_cmd_delay_t;

SIZE_CHECK(jerrycan_cmd_delay_t, 2);

typedef struct __attribute__((packed)) {
    uint8_t rsvd;
} jerrycan_cmd_fixed_xyz;

SIZE_CHECK(jerrycan_cmd_fixed_xyz, 1);

typedef struct __attribute__((packed)) {
    int32_t error;  // 0 is OK, negative is a system errno constant
} jerrycan_rsp_ack_t;

SIZE_CHECK(jerrycan_rsp_ack_t, 4);

typedef struct __attribute__((packed)) {
    jerrycan_cmd_type_t type;
    uint8_t dst_id;
    int64_t timestamp_ns;
    int64_t index;
    union {
        struct {
            union {
                jerrycan_cmd_estop_t estop;
                jerrycan_cmd_heartbeat_t heartbeat;
                jerrycan_cmd_status_t status;
                jerrycan_cmd_stepper_move_t stepper_move;
                jerrycan_cmd_servo_move_t servo_move;
                jerrycan_cmd_stepper_home_t stepper_home;
                jerrycan_cmd_stepper_fault_clear_t stepper_fault_clear;
                jerrycan_cmd_cfg_t cfg_write;
                jerrycan_cmd_cfg_t cfg_response;
                jerrycan_cmd_cfg_t cfg_read;
                jerrycan_cmd_stepper_status_t stepper_status;
                jerrycan_cmd_servo_status_t servo_status;
                jerrycan_cmd_pressure_read_t pressure_read;
                jerrycan_cmd_temp_hum_read_t temp_hum_read;
                jerrycan_cmd_gpio_read_t gpio_read;
                jerrycan_cmd_gpio_write_t gpio_write;
                jerrycan_cmd_tone_t tone;
                jerrycan_cmd_analog_out_t analog_out;
                jerrycan_cmd_load_cell_read_t load_cell_read;
                jerrycan_cmd_door_closed_t doors;
                jerrycan_cmd_audio_data_cmd_t audio_data_cmd;
                jerrycan_cmd_rgb_led_t rgb_led;
                jerrycan_cmd_load_cell_tare_t load_cell_tare;
                jerrycan_cmd_bootloader_command_t bootloader_command;
                jerrycan_cmd_bootloader_response_t bootloader_response;
                jerrycan_cmd_delay_t delay;
                jerrycan_cmd_fixed_xyz fixed_xyz;
                jerrycan_rsp_ack_t ack;
            };
            uuid_t uuid;
        };
        jerrycan_cmd_bootloader_data_t bootloader_data;
        jerrycan_cmd_audio_data_t audio_data;
        uint8_t payload[JERRYCAN_ACTUAL_PAYLOAD_SIZE];
    };
} jerrycan_msg_t;

BUILD_ASSERT(
    sizeof(jerrycan_msg_t) == (
            JERRYCAN_ACTUAL_PAYLOAD_SIZE
            + sizeof(jerrycan_cmd_type_t)
            + sizeof(uint8_t)
            + sizeof(int64_t)
            + sizeof(int64_t)
    ), "jerrycan_msg_t size should be max payload size + header fields size");

#define COMMAND_NOT_COMPLETE INT32_MIN
#define SEND_NO_ACKNOWLEDGEMENT INT32_MAX

typedef int (*jerrycan_rx_callback_fn_t)(const jerrycan_msg_t* msg);

typedef struct {
    jerrycan_rx_callback_fn_t func;
    jerrycan_cmd_type_t filter_msg_type;
    sys_snode_t node;
} jerrycan_rx_callback_t;

#ifdef __cplusplus
};
#endif
