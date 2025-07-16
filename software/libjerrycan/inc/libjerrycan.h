#pragma once

#include <variant>
#include <vector>
#include <optional>
#include "jerrycan_types.h"

class JerryCAN {
   public:
    JerryCAN() = default;

    [[nodiscard]] int Open();

    int Close();

    [[nodiscard]] int SendMessage(const jerrycan_msg_t &msg, uint16_t dst_id) const;

    int ReceiveMessage(jerrycan_msg_t &msg) const;

    std::vector<jerrycan_msg_t>
    ReceiveMessages(
        unsigned int max_count = 1,
        unsigned int collect_ms = 0
    ) const;

    [[nodiscard]] int Heartbeat() const;

    [[nodiscard]] int EStop(bool enable) const;

    [[nodiscard]] int StepperMove(uint8_t dst_id, uint8_t motor_id, float position, float max_velocity,
                                  float max_acceleration, abs_or_rel_t abs_or_rel, bool save, uuid_t uuid) const;

    [[nodiscard]] int ServoMove(uint8_t dst_id, uint8_t motor_id, float position, float max_velocity,
                                float max_acceleration, abs_or_rel_t abs_or_rel, uuid_t uuid) const;

    [[nodiscard]] int StepperHome(uint8_t dst_id, uint8_t motor_id, uuid_t uuid) const;

    [[nodiscard]] int CfgRead(uint8_t dst_id, const jerrycan_cmd_cfg_t &cfg) const;

    [[nodiscard]] int StepperCfgWrite(uint8_t dst_id, uint8_t motor_id, uint16_t microsteps, float steps_per_revolution,
                                      float motor_max_velocity, float motor_max_acceleration, float homing_velocity,
                                      bool flip_limit_orientation, uuid_t uuid) const;

    [[nodiscard]] int ServoCfgWrite(uint8_t dst_id, uint8_t motor_id, float min_position, float max_position,
                                    float min_pwm_duration_us, float max_pwm_duration_us, float motor_max_velocity,
                                    float motor_max_acceleration, uuid_t uuid) const;

    [[nodiscard]] int StepperCfgRead(uint8_t dst_id, uint8_t motor_id) const;

    [[nodiscard]] int ServoCfgRead(uint8_t dst_id, uint8_t motor_id) const;

    [[nodiscard]] int GPIOWrite(uint8_t dst_id, uint8_t instance, uint16_t gpio_idx, bool state, uuid_t uuid) const;

    [[nodiscard]] int ToneWrite(uint8_t dst_id, uint8_t instance, uint16_t frequency, uint16_t duration,
                                uuid_t uuid) const;

    [[nodiscard]] int AnalogOutWrite(uint8_t dst_id, uint8_t instance, uint16_t value_mv, uuid_t uuid) const;

    [[nodiscard]] int LoadCellTare(uint8_t dst_id, uint8_t instance, uuid_t uuid) const;

    [[nodiscard]] int RGBLEDWrite(uint8_t dst_id, uint8_t red, uint8_t green, uint8_t blue, uuid_t uuid) const;

    [[nodiscard]] int BootloaderCommand(uint8_t dst_id, jerrycan_bootloader_subcmd_t subcmd) const;

    [[nodiscard]] int BootloaderData(uint8_t dst_id, jerrycan_cmd_bootloader_data_t &data) const;

    [[nodiscard]] int Delay(uint8_t dst_id, uint16_t delay, uuid_t uuid) const;

    [[nodiscard]] int SendToFixedXYZ(uint8_t dst_id, uuid_t uuid) const;

   private:
    int _can_socket_handle;

    [[nodiscard]] int CfgWrite(uint8_t dst_id, const jerrycan_cmd_cfg_t &cfg, uuid_t uuid) const;
};
