#include "libjerrycan.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <spdlog/spdlog.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <iostream>
#include <map>

#include "jerrycan_types.h"

int JerryCAN::Open() {
    // Open the CAN socket
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        spdlog::error("Failed to open CAN socket: {}", errno);
        return -EIO;
    }

    spdlog::info("Opened CAN socket: {}", s);

    // Set a timeout on the socket so reads don't block forever
    timeval tv = {0};
    tv.tv_sec = 0;
    tv.tv_usec = 1000;  // 1ms
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    constexpr auto enable = 1U;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable));

    // Find the CAN interface index
    ifreq ifr{};
    strcpy(ifr.ifr_name, "can0");  // FIXME: Make this configurable
    ioctl(s, SIOCGIFINDEX, &ifr);

    // Bind to the CAN sockets
    struct sockaddr_can addr = {0};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        spdlog::error("Failed to bind CAN socket: {}", errno);
        return -EIO;
    }

    spdlog::info("Bound CAN socket");

    _can_socket_handle = s;

    return 0;
}

/* -------------------------------------------------------------------------- */

int JerryCAN::Close() {
    // Close the CAN socket
    if (close(_can_socket_handle) < 0) {
        spdlog::error("Failed to close CAN socket: {}", errno);
        return -errno;
    }

    spdlog::info("Closed CAN socket: {}", _can_socket_handle);
    _can_socket_handle = -1;

    return 0;
}

/* -------------------------------------------------------------------------- */

// Return the payload size for a given message type
static uint8_t jerrycan_msg_get_payload_size(const jerrycan_cmd_type_t msg_type) {
    static const std::map<jerrycan_cmd_type_t, ssize_t> jerrycan_size_map = {
        {JERRYCAN_CMD_ESTOP, sizeof(jerrycan_cmd_estop_t)},
        {JERRYCAN_CMD_HEARTBEAT, sizeof(jerrycan_cmd_heartbeat_t)},
        {JERRYCAN_CMD_STATUS, sizeof(jerrycan_cmd_status_t)},
        {JERRYCAN_CMD_STEPPER_MOVE, sizeof(jerrycan_cmd_stepper_move_t)},
        {JERRYCAN_CMD_SERVO_MOVE, sizeof(jerrycan_cmd_servo_move_t)},
        {JERRYCAN_CMD_STEPPER_HOME, sizeof(jerrycan_cmd_stepper_home_t)},
        {JERRYCAN_CMD_CFG_WRITE, sizeof(jerrycan_cmd_cfg_t)},
        {JERRYCAN_CMD_CFG_RESPONSE, sizeof(jerrycan_cmd_cfg_t)},
        {JERRYCAN_CMD_CFG_READ, sizeof(jerrycan_cmd_cfg_t)},
        {JERRYCAN_CMD_STEPPER_STATUS, sizeof(jerrycan_cmd_stepper_status_t)},
        {JERRYCAN_CMD_SERVO_STATUS, sizeof(jerrycan_cmd_servo_status_t)},
        {JERRYCAN_CMD_PRESSURE_READ, sizeof(jerrycan_cmd_pressure_read_t)},
        {JERRYCAN_CMD_TEMP_HUM_READ, sizeof(jerrycan_cmd_temp_hum_read_t)},
        {JERRYCAN_CMD_GPIO_READ, sizeof(jerrycan_cmd_gpio_read_t)},
        {JERRYCAN_CMD_GPIO_WRITE, sizeof(jerrycan_cmd_gpio_write_t)},
        {JERRYCAN_CMD_TONE, sizeof(jerrycan_cmd_tone_t)},
        {JERRYCAN_CMD_ANALOG_OUT, sizeof(jerrycan_cmd_analog_out_t)},
        {JERRYCAN_CMD_LOAD_CELL_READ, sizeof(jerrycan_cmd_load_cell_read_t)},
        {JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_BEGIN, sizeof(jerrycan_cmd_audio_data_cmd_t)},
        {JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_CONT, sizeof(jerrycan_cmd_audio_data_t)},
        {JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_END, sizeof(jerrycan_cmd_audio_data_cmd_t)},
        {JERRYCAN_CMD_LOAD_CELL_TARE, sizeof(jerrycan_cmd_load_cell_tare_t)},
        {JERRYCAN_CMD_RGB_LED, sizeof(jerrycan_cmd_rgb_led_t)},
        {JERRYCAN_CMD_DOOR_SENSOR, sizeof(jerrycan_cmd_door_closed_t)},
        {JERRYCAN_CMD_BOOTLOADER_COMMAND, sizeof(jerrycan_cmd_bootloader_command_t)},
        {JERRYCAN_CMD_BOOTLOADER_RESPONSE, sizeof(jerrycan_cmd_bootloader_response_t)},
        {JERRYCAN_CMD_BOOTLOADER_DATA, sizeof(jerrycan_cmd_bootloader_data_t)},
        {JERRYCAN_CMD_DELAY, sizeof(jerrycan_cmd_delay_t)},
        {JERRYCAN_CMD_FIXED_XYZ, sizeof(jerrycan_cmd_fixed_xyz)},
        {JERRYCAN_RSP_ACK, sizeof(jerrycan_rsp_ack_t)},
    };

    try {
        return jerrycan_size_map.at(msg_type);
    } catch (std::out_of_range &) {
        return 0;
    }
}

/* -------------------------------------------------------------------------- */

int JerryCAN::SendMessage(const jerrycan_msg_t &msg, const uint16_t dst_id) const {
    // Send the message
    canfd_frame frame = {0};
    uint8_t payload_size = jerrycan_msg_get_payload_size(msg.type);
    frame.can_id = ((msg.type & 0x3F) << 5) | (dst_id & 0x1F);

    memcpy(frame.data, msg.payload, payload_size);
    if (payload_size < sizeof(msg.payload)) {
        memcpy(frame.data + payload_size, &msg.uuid, sizeof(msg.uuid));
        payload_size += sizeof(msg.uuid);
    }

    frame.len = payload_size;

    // For payload sizes > 8, DLC no longer maps 1:1 to the number of bytes in the payload
    // If the actual payload size is less than the number of bytes indicated by DLC, pad the rest with 0
    // const uint8_t dlc_bytes = can_dlc_to_bytes(can_bytes_to_dlc(payload_size));
    // memset(&frame.data[payload_size], 0, dlc_bytes - payload_size);

    if (write(_can_socket_handle, &frame, sizeof(frame)) < 0) {
        spdlog::error("Failed to send CAN message: {}", errno);
        return -errno;
    }

    return 0;
}

/* -------------------------------------------------------------------------- */

int JerryCAN::ReceiveMessage(jerrycan_msg_t &msg) const {
    // Receive a message
    canfd_frame frame = {0};
    int read_res = read(_can_socket_handle, &frame, sizeof(frame));
    if (read_res <= 0) {
        if (read_res == 0) {
            return -EAGAIN;
        }
        if (errno != EAGAIN && errno != EINTR) {
            spdlog::error("Failed to receive CAN message: {}", errno);
        }
        return -errno;
    }

    msg.type = static_cast<jerrycan_cmd_type_t>((frame.can_id >> 5) & 0x3F);
    msg.dst_id = frame.can_id & 0x1F;
    const uint8_t msg_len = jerrycan_msg_get_payload_size(msg.type);
    memcpy(msg.payload, frame.data, msg_len);
    if (msg_len < sizeof(msg.payload)) {
        memcpy(&msg.uuid, frame.data + msg_len, sizeof(msg.uuid));
    }

    return 0;
}

/* -------------------------------------------------------------------------- */

std::vector<jerrycan_msg_t>
JerryCAN::ReceiveMessages(
    unsigned int max_count,
    unsigned int collect_ms
) const {
    jerrycan_msg_t msg;
    std::vector<jerrycan_msg_t> res_vec;
    auto end = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(collect_ms);
    while (true) {
        // ReceiveMessage->read is currently blocking for 1 ms when no message immediatelly available.
        const auto ret = this->ReceiveMessage(msg);  // so no need sleep between calls
        if (ret == 0) {
            res_vec.push_back(msg);
            if (max_count > 0 && res_vec.size() >= max_count) {
                break;
            }
        } else if (ret == -EINTR) {
            break;
        } else if (ret != -EAGAIN && ret != -EWOULDBLOCK) {
            // neither EINTR or EAGAIN, should raise/throw an error instead
            break;
        }
        if (collect_ms == 0 || std::chrono::high_resolution_clock::now() > end) {
            break;
        }
    }
    return res_vec;
}

/* -------------------------------------------------------------------------- */

int JerryCAN::Heartbeat() const {
    // Send a heartbeat message
    constexpr jerrycan_msg_t msg = {
        .type = JERRYCAN_CMD_HEARTBEAT,
        .heartbeat =
            {
                .rsvd = 0xFF,
            },
    };

    return SendMessage(msg, 0x1F);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::EStop(const bool enable) const {
    // Send an emergency stop message
    jerrycan_msg_t msg = {
        .type = JERRYCAN_CMD_ESTOP,
        .estop =
            {
                .rsvd = enable,
            },
    };

    msg.uuid = 0;

    return SendMessage(msg, 0x1F);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::StepperMove(uint8_t dst_id, uint8_t motor_id, float position, float max_velocity, float max_acceleration,
                          abs_or_rel_t abs_or_rel, bool save, uuid_t uuid) const {
    // Send a stepper move message
    jerrycan_msg_t msg;
    msg.type = JERRYCAN_CMD_STEPPER_MOVE;
    msg.uuid = uuid;

    msg.stepper_move.motor_id = motor_id;
    msg.stepper_move.save = save;
    msg.stepper_move.rsvd0 = 0;
    msg.stepper_move.abs_or_rel = abs_or_rel;
    msg.stepper_move.position = position;
    msg.stepper_move.max_velocity = max_velocity;
    msg.stepper_move.max_acceleration = max_acceleration;

    return SendMessage(msg, dst_id);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::ServoMove(uint8_t dst_id, uint8_t motor_id, float position, float max_velocity, float max_acceleration,
                        abs_or_rel_t abs_or_rel, uuid_t uuid) const {
    // Send a servo move message
    jerrycan_msg_t msg;
    msg.type = JERRYCAN_CMD_SERVO_MOVE;
    msg.uuid = uuid;

    msg.servo_move.motor_id = motor_id;
    msg.servo_move.rsvd0 = 0;
    msg.servo_move.abs_or_rel = abs_or_rel;
    msg.servo_move.position = position;
    msg.servo_move.max_velocity = max_velocity;
    msg.servo_move.max_acceleration = max_acceleration;

    return SendMessage(msg, dst_id);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::StepperHome(const uint8_t dst_id, const uint8_t motor_id, const uuid_t uuid) const {
    // Send a stepper home message
    jerrycan_msg_t msg = {
        .type = JERRYCAN_CMD_STEPPER_HOME,
        .stepper_home =
            {
                .motor_id = motor_id,
            },
    };

    msg.uuid = uuid;

    return SendMessage(msg, dst_id);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::CfgWrite(uint8_t dst_id, const jerrycan_cmd_cfg_t &cfg, uuid_t uuid) const {
    // Send a Configuration Write Message
    jerrycan_msg_t msg = {
        .type = JERRYCAN_CMD_CFG_WRITE,
        .cfg_write = cfg,
    };

    msg.uuid = uuid;

    return SendMessage(msg, dst_id);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::CfgRead(uint8_t dst_id, const jerrycan_cmd_cfg_t &cfg) const {
    // Send a Configuration Read Message
    jerrycan_msg_t msg = {
        .type = JERRYCAN_CMD_CFG_READ,
        .cfg_read = cfg,
    };

    msg.uuid = 0;

    return SendMessage(msg, dst_id);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::StepperCfgWrite(const uint8_t dst_id, const uint8_t motor_id, const uint16_t microsteps,
                              const float steps_per_revolution, const float motor_max_velocity,
                              const float motor_max_acceleration, const float homing_velocity,
                              const bool flip_limit_orientation, const uuid_t uuid) const {
    jerrycan_cmd_cfg_t cfg_write;
    cfg_write.type = JERRYCAN_CFG_STEPPER;

    cfg_write.stepper.motor_id = motor_id;
    cfg_write.stepper.flip_limit_orientation = flip_limit_orientation;
    cfg_write.stepper.microsteps = microsteps;
    cfg_write.stepper.steps_per_revolution = steps_per_revolution;
    cfg_write.stepper.motor_max_velocity = motor_max_velocity;
    cfg_write.stepper.motor_max_acceleration = motor_max_acceleration;
    cfg_write.stepper.homing_velocity = homing_velocity;

    return CfgWrite(dst_id, cfg_write, uuid);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::ServoCfgWrite(const uint8_t dst_id, const uint8_t motor_id, const float min_position,
                            const float max_position, const float min_pwm_duration_us, const float max_pwm_duration_us,
                            const float motor_max_velocity, const float motor_max_acceleration,
                            const uuid_t uuid) const {
    const jerrycan_cmd_cfg_t cfg_write = {.type = JERRYCAN_CFG_SERVO,
                                          .servo = {
                                              .motor_id = motor_id,
                                              .min_position = min_position,
                                              .max_position = max_position,
                                              .min_pwm_duration_us = min_pwm_duration_us,
                                              .max_pwm_duration_us = max_pwm_duration_us,
                                              .motor_max_velocity = motor_max_velocity,
                                              .motor_max_acceleration = motor_max_acceleration,
                                          }};

    return CfgWrite(dst_id, cfg_write, uuid);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::StepperCfgRead(const uint8_t dst_id, const uint8_t motor_id) const {
    const jerrycan_cmd_cfg_t cfg_read = {
        .type = JERRYCAN_CFG_STEPPER,
        .stepper =
            {
                .motor_id = motor_id,
            },
    };

    return CfgRead(dst_id, cfg_read);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::ServoCfgRead(const uint8_t dst_id, const uint8_t motor_id) const {
    const jerrycan_cmd_cfg_t cfg_read = {
        .type = JERRYCAN_CFG_SERVO,
        .servo =
            {
                .motor_id = motor_id,
            },
    };

    return CfgRead(dst_id, cfg_read);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::GPIOWrite(uint8_t dst_id, uint8_t instance, uint16_t gpio_idx, bool state, uuid_t uuid) const {
    // Send a GPIO Write Message
    jerrycan_msg_t msg = {
        .type = JERRYCAN_CMD_GPIO_WRITE,
        .gpio_write =
            {
                .instance = instance,
                .gpio_idx = gpio_idx,
                .state = state,
            },
    };

    msg.uuid = uuid;

    return SendMessage(msg, dst_id);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::ToneWrite(uint8_t dst_id, uint8_t instance, uint16_t frequency, uint16_t duration, uuid_t uuid) const {
    // Send a GPIO Write Message
    jerrycan_msg_t msg = {
        .type = JERRYCAN_CMD_TONE,
        .tone =
            {
                .instance = instance,
                .frequency_hz = frequency,
                .duration_ms = duration,
            },
    };

    msg.uuid = uuid;

    return SendMessage(msg, dst_id);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::AnalogOutWrite(const uint8_t dst_id, const uint8_t instance, const uint16_t value_mv,
                             const uuid_t uuid) const {
    // Send an Analog Out Message
    jerrycan_msg_t msg = {
        .type = JERRYCAN_CMD_ANALOG_OUT,
        .analog_out =
            {
                .instance = instance,
                .value_mv = value_mv,
            },
    };

    msg.uuid = uuid;

    return SendMessage(msg, dst_id);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::LoadCellTare(uint8_t dst_id, uint8_t instance, uuid_t uuid) const {
    jerrycan_msg_t msg = {
        .type = JERRYCAN_CMD_LOAD_CELL_TARE,
        .load_cell_tare =
            {
                .instance = instance,
            },
    };

    msg.uuid = uuid;

    return SendMessage(msg, dst_id);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::RGBLEDWrite(uint8_t dst_id, uint8_t red, uint8_t green, uint8_t blue, uuid_t uuid) const {
    jerrycan_msg_t msg = {
        .type = JERRYCAN_CMD_RGB_LED,
        .rgb_led =
            {
                .red = red,
                .green = green,
                .blue = blue,
            },
    };

    msg.uuid = uuid;

    return SendMessage(msg, dst_id);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::BootloaderCommand(uint8_t dst_id, jerrycan_bootloader_subcmd_t subcmd) const {
    jerrycan_msg_t msg = {
        .type = JERRYCAN_CMD_BOOTLOADER_COMMAND,
        .bootloader_command =
            {
                .type = subcmd,
            },
    };

    msg.uuid = 0;

    return SendMessage(msg, dst_id);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::BootloaderData(uint8_t dst_id, jerrycan_cmd_bootloader_data_t &data) const {
    jerrycan_msg_t msg = {
        .type = JERRYCAN_CMD_BOOTLOADER_DATA,
        // don't set UUID!
    };

    msg.bootloader_data = data;

    return SendMessage(msg, dst_id);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::Delay(const uint8_t dst_id, const uint16_t delay, const uuid_t uuid) const {
    jerrycan_msg_t msg = {
        .type = JERRYCAN_CMD_DELAY,
        .delay =
            {
                .delay = delay,
            },
    };

    msg.uuid = uuid;

    return SendMessage(msg, dst_id);
}

/* -------------------------------------------------------------------------- */

int JerryCAN::SendToFixedXYZ(const uint8_t dst_id, const uuid_t uuid) const {
    const jerrycan_msg_t msg = {
        .type = JERRYCAN_CMD_FIXED_XYZ,
        .uuid = uuid,
    };

    return SendMessage(msg, dst_id);
}