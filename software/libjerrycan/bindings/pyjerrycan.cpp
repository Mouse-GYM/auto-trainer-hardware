#include <optional> // For std::optional
#include <variant>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h> // For binding specific STL containers if needed

#include <libjerrycan.h>

namespace py = pybind11;

/* clang-format off */
PYBIND11_MODULE(pyjerrycan, m) {
    m.doc() = "JerryCAN Host Interface";
    py::class_<JerryCAN>(m, "JerryCAN")
        .def(py::init<>())
        .def("Open", &JerryCAN::Open, py::call_guard<py::gil_scoped_release>())
        .def("Close", &JerryCAN::Close, py::call_guard<py::gil_scoped_release>())
        .def("SendMessage", &JerryCAN::SendMessage, py::arg("msg"), py::arg("dst_id"),
             py::call_guard<py::gil_scoped_release>())
        .def("ReceiveMessages", &JerryCAN::ReceiveMessages, py::arg("max_count") = 1, py::arg("collect_ms") = 0, py::call_guard<py::gil_scoped_release>())
        .def("ReceiveMessage", [](const JerryCAN &j) {
            jerrycan_msg_t msg;
            const auto ret = j.ReceiveMessage(msg);
            return ret < 0 ? std::nullopt : std::make_optional(msg);
        }, py::call_guard<py::gil_scoped_release>())
        .def("Heartbeat", &JerryCAN::Heartbeat, py::call_guard<py::gil_scoped_release>())
        .def("EStop", &JerryCAN::EStop, py::call_guard<py::gil_scoped_release>())
        .def("StepperMove", &JerryCAN::StepperMove,
             py::arg("dst_id"), py::arg("motor_id"), py::arg("position"), py::arg("max_velocity"), py::arg("max_acceleration"), py::arg("abs_or_rel"), py::arg("save"), py::arg("uuid"),
             py::call_guard<py::gil_scoped_release>())
        .def("ServoMove", &JerryCAN::ServoMove, py::arg("dst_id"), py::arg("motor_id"), py::arg("position"), py::arg("max_velocity"), py::arg("max_acceleration"), py::arg("abs_or_rel"), py::arg("uuid"),
             py::call_guard<py::gil_scoped_release>())
        .def("StepperHome", &JerryCAN::StepperHome, py::arg("dst_id"), py::arg("motor_id"), py::arg("uuid"),
             py::call_guard<py::gil_scoped_release>())
        .def("StepperCfgWrite", &JerryCAN::StepperCfgWrite, py::arg("dst_id"), py::arg("motor_id"), py::arg("microsteps"), py::arg("steps_per_revolution"), py::arg("motor_max_velocity"), py::arg("motor_max_acceleration"), py::arg("homing_velocity"), py::arg("flip_limit_orientation"), py::arg("uuid"),
             py::call_guard<py::gil_scoped_release>())
        .def("ServoCfgWrite", &JerryCAN::ServoCfgWrite, py::arg("dst_id"), py::arg("motor_id"), py::arg("min_position"), py::arg("max_position"), py::arg("min_pwm_duration_us"), py::arg("max_pwm_duration_us"), py::arg("motor_max_velocity"), py::arg("motor_max_acceleration"), py::arg("uuid"),
             py::call_guard<py::gil_scoped_release>())
        .def("StepperCfgRead", &JerryCAN::StepperCfgRead, py::arg("dst_id"), py::arg("motor_id"), py::call_guard<py::gil_scoped_release>())
        .def("ServoCfgRead", &JerryCAN::ServoCfgRead, py::arg("dst_id"), py::arg("motor_id"), py::call_guard<py::gil_scoped_release>())
        .def("CfgRead", &JerryCAN::CfgRead, py::arg("dst_id"), py::arg("cfg"), py::call_guard<py::gil_scoped_release>())
        .def("GPIOWrite", &JerryCAN::GPIOWrite, py::arg("dst_id"), py::arg("instance"), py::arg("gpio_idx"), py::arg("state"), py::arg("uuid"), py::call_guard<py::gil_scoped_release>())
        .def("ToneWrite", &JerryCAN::ToneWrite, py::arg("dst_id"), py::arg("instance"), py::arg("frequency"), py::arg("duration"), py::arg("uuid"), py::call_guard<py::gil_scoped_release>())
        .def("AnalogOutWrite", &JerryCAN::AnalogOutWrite, py::arg("dst_id"), py::arg("instance"), py::arg("value_mv"), py::arg("uuid"), py::call_guard<py::gil_scoped_release>())
        .def("LoadCellTare", &JerryCAN::LoadCellTare, py::arg("dst_id"), py::arg("instance"), py::arg("uuid"), py::call_guard<py::gil_scoped_release>())
        .def("RGBLEDWrite", &JerryCAN::RGBLEDWrite, py::arg("dst_id"), py::arg("red"), py::arg("green"), py::arg("blue"), py::arg("uuid"), py::call_guard<py::gil_scoped_release>())
        .def("BootloaderCommand", &JerryCAN::BootloaderCommand, py::arg("dst_id"), py::arg("subcmd"), py::call_guard<py::gil_scoped_release>())
        .def("Delay", &JerryCAN::Delay, py::arg("dst_id"), py::arg("delay"), py::arg("uuid"), py::call_guard<py::gil_scoped_release>())
        .def("SendToFixedXYZ", &JerryCAN::SendToFixedXYZ, py::arg("dst_id"), py::arg("uuid"), py::call_guard<py::gil_scoped_release>())
    ;

    py::class_<jerrycan_msg_t>(m, "JerryCANMsg")
        .def(py::init<>())
        .def_readwrite("type", &jerrycan_msg_t::type)
        .def_readwrite("dst_id", &jerrycan_msg_t::dst_id)
        .def_readwrite("uuid", &jerrycan_msg_t::uuid)
        .def_readwrite("estop", &jerrycan_msg_t::estop)
        .def_readwrite("status", &jerrycan_msg_t::status)
        .def_readwrite("heartbeat", &jerrycan_msg_t::heartbeat)
        .def_readwrite("stepper_move", &jerrycan_msg_t::stepper_move)
        .def_readwrite("servo_move", &jerrycan_msg_t::servo_move)
        .def_readwrite("stepper_home", &jerrycan_msg_t::stepper_home)
        .def_readwrite("cfg_response", &jerrycan_msg_t::cfg_response)
        .def_readwrite("cfg_read", &jerrycan_msg_t::cfg_read)
        .def_readwrite("cfg_write", &jerrycan_msg_t::cfg_write)
        .def_readwrite("stepper_status", &jerrycan_msg_t::stepper_status)
        .def_readwrite("servo_status", &jerrycan_msg_t::servo_status)
        .def_readwrite("pressure_read", &jerrycan_msg_t::pressure_read)
        .def_readwrite("temp_hum_read", &jerrycan_msg_t::temp_hum_read)
        .def_readwrite("gpio_read", &jerrycan_msg_t::gpio_read)
        .def_readwrite("gpio_write", &jerrycan_msg_t::gpio_write)
        .def_readwrite("tone", &jerrycan_msg_t::tone)
        .def_readwrite("analog_out", &jerrycan_msg_t::analog_out)
        .def_readwrite("load_cell_read", &jerrycan_msg_t::load_cell_read)
        .def_readwrite("load_cell_tare", &jerrycan_msg_t::load_cell_tare)
        .def_readwrite("rgb_led", &jerrycan_msg_t::rgb_led)
        .def_readwrite("doors", &jerrycan_msg_t::doors)
        .def_readwrite("audio_data_cmd", &jerrycan_msg_t::audio_data_cmd)
        .def_readwrite("audio_data", &jerrycan_msg_t::audio_data)
        .def_readwrite("bootloader_command", &jerrycan_msg_t::bootloader_command)
        .def_readwrite("bootloader_response", &jerrycan_msg_t::bootloader_response)
        .def_readwrite("bootloader_data", &jerrycan_msg_t::bootloader_data)
        .def_readwrite("delay", &jerrycan_msg_t::delay)
        .def_readwrite("fixed_xyz", &jerrycan_msg_t::fixed_xyz)
        .def_readwrite("ack", &jerrycan_msg_t::ack)
    ;

    py::class_<jerrycan_cmd_status_t>(m, "Status")
        .def(py::init<>())
        .def_property("estop_active",
            [](const jerrycan_cmd_status_t &a) { return a.estop_active; },
            [](jerrycan_cmd_status_t &a, const uint8_t v) { a.estop_active = v; })
        .def_property("limit_switch0",
            [](const jerrycan_cmd_status_t &a) { return a.limit_switch0; },
            [](jerrycan_cmd_status_t &a, const uint8_t v) { a.limit_switch0 = v; })
        .def_property("limit_switch1",
            [](const jerrycan_cmd_status_t &a) { return a.limit_switch1; },
            [](jerrycan_cmd_status_t &a, const uint8_t v) { a.limit_switch1 = v; })
        .def_property("limit_switch2",
            [](const jerrycan_cmd_status_t &a) { return a.limit_switch2; },
            [](jerrycan_cmd_status_t &a, const uint8_t v) { a.limit_switch2 = v; })
        .def_property("button0",
            [](const jerrycan_cmd_status_t &a) { return a.button0; },
            [](jerrycan_cmd_status_t &a, const uint8_t v) { a.button0 = v; })
        .def_property("stepper_status0",
            [](const jerrycan_cmd_status_t &a) { return a.stepper_status0; },
            [](jerrycan_cmd_status_t &a, const uint8_t v) { a.stepper_status0 = v; })
        .def_property("stepper_status1",
            [](const jerrycan_cmd_status_t &a) { return a.stepper_status1; },
            [](jerrycan_cmd_status_t &a, const uint8_t v) { a.stepper_status1 = v; })
        .def_property("stepper_status2",
            [](const jerrycan_cmd_status_t &a) { return a.stepper_status2; },
            [](jerrycan_cmd_status_t &a, const uint8_t v) { a.stepper_status2 = v; })
        .def_property("servo_status0",
            [](const jerrycan_cmd_status_t &a) { return a.servo_status0; },
            [](jerrycan_cmd_status_t &a, const uint8_t v) { a.servo_status0 = v; })
        .def_property("servo_status1",
            [](const jerrycan_cmd_status_t &a) { return a.servo_status1; },
            [](jerrycan_cmd_status_t &a, const uint8_t v) { a.servo_status1 = v; })
        .def_property("servo_status2",
            [](const jerrycan_cmd_status_t &a) { return a.servo_status2; },
            [](jerrycan_cmd_status_t &a, const uint8_t v) { a.servo_status2 = v; })
    ;

    py::class_<jerrycan_cmd_cfg_t> cmd_cfg(m, "JerryCANCfgMsg");
    cmd_cfg.def(py::init<>())
        .def_readwrite("type", &jerrycan_cmd_cfg_t::type)
        .def_readwrite("servo", &jerrycan_cmd_cfg_t::servo)
        .def_readwrite("stepper", &jerrycan_cmd_cfg_t::stepper)
    ;

    py::enum_<jerrycan_cfg_type_t>(cmd_cfg, "Type")
        .value("STEPPER", JERRYCAN_CFG_STEPPER)
        .value("SERVO", JERRYCAN_CFG_SERVO)
        .export_values()
    ;

    py::class_<jerrycan_servo_cfg_t>(cmd_cfg, "ServoCfg")
        .def(py::init<>())
        .def_property("motor_id",
            // This is a workaround for setting bitfields
            [](const jerrycan_servo_cfg_t &a) { return a.motor_id; },
            [](jerrycan_servo_cfg_t &a, const uint8_t v) { a.motor_id = v; })
        .def_property("error",
            [](const jerrycan_servo_cfg_t &a) { return a.error; },
            [](jerrycan_servo_cfg_t &a, const uint8_t v) { a.error = v; })
        .def_readwrite("min_position", &jerrycan_servo_cfg_t::min_position)
        .def_readwrite("max_position", &jerrycan_servo_cfg_t::max_position)
        .def_readwrite("min_pwm_duration_us", &jerrycan_servo_cfg_t::min_pwm_duration_us)
        .def_readwrite("max_pwm_duration_us", &jerrycan_servo_cfg_t::max_pwm_duration_us)
        .def_readwrite("motor_max_velocity", &jerrycan_servo_cfg_t::motor_max_velocity)
        .def_readwrite("motor_max_acceleration", &jerrycan_servo_cfg_t::motor_max_acceleration)
    ;


    py::class_<jerrycan_stepper_cfg_t>(cmd_cfg, "StepperCfg")
        .def(py::init<>())
        .def_property("motor_id",
            [](const jerrycan_stepper_cfg_t &a) { return a.motor_id; },
            [](jerrycan_stepper_cfg_t &a, const uint8_t v) { a.motor_id = v; })
        .def_property("flip_limit_orientation",
            [](const jerrycan_stepper_cfg_t &a) { return a.flip_limit_orientation; },
            [](jerrycan_stepper_cfg_t &a, const uint8_t flip_limit_orientation) { a.flip_limit_orientation = flip_limit_orientation; })
        .def_readwrite("microsteps", &jerrycan_stepper_cfg_t::microsteps)
        .def_readwrite("steps_per_revolution", &jerrycan_stepper_cfg_t::steps_per_revolution)
        .def_readwrite("motor_max_velocity", &jerrycan_stepper_cfg_t::motor_max_velocity)
        .def_readwrite("motor_max_acceleration", &jerrycan_stepper_cfg_t::motor_max_acceleration)
        .def_readwrite("homing_velocity", &jerrycan_stepper_cfg_t::homing_velocity)
    ;

    py::class_<jerrycan_cmd_pressure_read_t>(m, "PressureRead")
        .def(py::init<>())
        .def_property("instance",
            [](const jerrycan_cmd_pressure_read_t &a) { return a.instance; },
            [](jerrycan_cmd_pressure_read_t &a, const uint8_t v) { a.instance = v; })
        .def_readwrite("pressure", &jerrycan_cmd_pressure_read_t::pressure)
    ;

    py::class_<jerrycan_cmd_temp_hum_read_t>(m, "TempHumRead")
        .def(py::init<>())
        .def_property("instance",
            [](const jerrycan_cmd_temp_hum_read_t &a) { return a.instance; },
            [](jerrycan_cmd_temp_hum_read_t &a, const uint8_t v) { a.instance = v; })
        .def_readwrite("temperature", &jerrycan_cmd_temp_hum_read_t::temperature)
        .def_readwrite("humidity", &jerrycan_cmd_temp_hum_read_t::humidity)
    ;

    py::class_<jerrycan_cmd_gpio_read_t>(m, "GPIORead")
        .def(py::init<>())
        .def_property("instance",
            [](const jerrycan_cmd_gpio_read_t &a) { return a.instance; },
            [](jerrycan_cmd_gpio_read_t &a, const uint8_t v) { a.instance = v; })
        .def_property("state",
            [](const jerrycan_cmd_gpio_read_t &a) { return a.state; },
            [](jerrycan_cmd_gpio_read_t &a, const uint8_t v) { a.state = v; })
    ;

    py::class_<jerrycan_cmd_gpio_write_t>(m, "GPIOWrite")
        .def(py::init<>())
        .def_property("instance",
            [](const jerrycan_cmd_gpio_write_t &a) { return a.instance; },
            [](jerrycan_cmd_gpio_write_t &a, const uint8_t v) { a.instance = v; })
        .def_property("gpio_idx",
            [](const jerrycan_cmd_gpio_write_t &a) { return a.gpio_idx; },
            [](jerrycan_cmd_gpio_write_t &a, const uint8_t v) { a.gpio_idx = v; })
        .def_property("state",
            [](const jerrycan_cmd_gpio_write_t &a) { return a.state; },
            [](jerrycan_cmd_gpio_write_t &a, const uint8_t v) { a.state = v; })
    ;

    py::class_<jerrycan_cmd_tone_t>(m, "Tone")
        .def(py::init<>())
        .def_readwrite("instance", &jerrycan_cmd_tone_t::instance)
        .def_readwrite("frequency_hz", &jerrycan_cmd_tone_t::frequency_hz)
        .def_readwrite("duration_ms", &jerrycan_cmd_tone_t::duration_ms)
    ;

    py::class_<jerrycan_cmd_analog_out_t>(m, "AnalogOut")
        .def(py::init<>())
        .def_readwrite("instance", &jerrycan_cmd_analog_out_t::instance)
        .def_readwrite("value_mv", &jerrycan_cmd_analog_out_t::value_mv)
    ;

    py::class_<jerrycan_cmd_load_cell_read_t>(m, "LoadCellRead")
        .def(py::init<>())
        .def_readwrite("instance", &jerrycan_cmd_load_cell_read_t::instance)
        .def_readwrite("load_mv", &jerrycan_cmd_load_cell_read_t::load_mv)
    ;
    
    py::class_<jerrycan_cmd_stepper_home_t>(m, "StepperHome")
        .def(py::init<>())
        .def_property("motor_id",
            [](const jerrycan_cmd_stepper_home_t &a) { return a.motor_id; },
            [](jerrycan_cmd_stepper_home_t &a, const uint8_t v) { a.motor_id = v; })
    ;

    py::class_<jerrycan_cmd_stepper_status_t>(m, "StepperStatus")
        .def(py::init<>())
        .def_readwrite("motor_id", &jerrycan_cmd_stepper_status_t::motor_id)
        .def_readwrite("status", &jerrycan_cmd_stepper_status_t::status)
        .def_readwrite("homing_status", &jerrycan_cmd_stepper_status_t::homing_status)
        .def_readwrite("position", &jerrycan_cmd_stepper_status_t::position)
        .def_readwrite("limit_switch", &jerrycan_cmd_stepper_status_t::limit_switch)
    ;

    py::class_<jerrycan_cmd_servo_status_t>(m, "ServoStatus")
        .def(py::init<>())
        .def_readwrite("motor_id", &jerrycan_cmd_servo_status_t::motor_id)
        .def_readwrite("status", &jerrycan_cmd_servo_status_t::status)
        .def_readwrite("position", &jerrycan_cmd_servo_status_t::position)
    ;

    py::class_<jerrycan_cmd_load_cell_tare_t>(m, "LoadCellTare")
        .def(py::init<>())
        .def_readwrite("instance", &jerrycan_cmd_load_cell_tare_t::instance)
    ;

    py::enum_<abs_or_rel_t>(m, "AbsOrRel")
        .value("ABSOLUTE", JERRYCAN_MOVE_ABSOLUTE)
        .value("RELATIVE", JERRYCAN_MOVE_RELATIVE)
        .export_values()
    ;

    py::class_<jerrycan_cmd_rgb_led_t>(m, "RGBLED")
        .def(py::init<>())
        .def_readwrite("red", &jerrycan_cmd_rgb_led_t::red)
        .def_readwrite("green", &jerrycan_cmd_rgb_led_t::green)
        .def_readwrite("blue", &jerrycan_cmd_rgb_led_t::blue)
    ;
    
    py::class_<jerrycan_cmd_door_closed_t>(m, "Doors")
        .def(py::init<>())
        .def_property("door1",
            [](const jerrycan_cmd_door_closed_t &a) { return a.door1; },
            [](jerrycan_cmd_door_closed_t &a, const uint8_t v) { a.door1 = v; })
    		.def_property("door2",
            [](const jerrycan_cmd_door_closed_t &a) { return a.door2; },
            [](jerrycan_cmd_door_closed_t &a, const uint8_t v) { a.door2 = v; })
    		.def_property("door3",
            [](const jerrycan_cmd_door_closed_t &a) { return a.door3; },
            [](jerrycan_cmd_door_closed_t &a, const uint8_t v) { a.door3 = v; })
    		.def_property("ext_button",
            [](const jerrycan_cmd_door_closed_t &a) { return a.external_button; },
            [](jerrycan_cmd_door_closed_t &a, const uint8_t v) { a.external_button = v; })
    ;

    py::class_<jerrycan_cmd_audio_data_cmd_t>(m, "AudioDataCmd")
        .def(py::init<>())
        .def_readwrite("stream_id", &jerrycan_cmd_audio_data_cmd_t::stream_id)
    ;

    py::class_<jerrycan_cmd_audio_data_t>(m, "AudioData")
        .def(py::init<>())
        .def_property("magnitudes",
            [](const jerrycan_cmd_audio_data_t &a) { return std::vector<float>(a.magnitudes, a.magnitudes + sizeof(jerrycan_cmd_audio_data_t)/sizeof(float)); },
            [](jerrycan_cmd_audio_data_t &a, const std::vector<float> &v) { 
                if (v.size() != sizeof(jerrycan_cmd_audio_data_t)/sizeof(float)) {
                    throw std::runtime_error("Size mismatch: Expected array of size " + std::to_string(sizeof(jerrycan_cmd_audio_data_t)/sizeof(float)));
                }
                std::memcpy(a.magnitudes, v.data(), sizeof(jerrycan_cmd_audio_data_t)); })
    ;

    py::class_<jerrycan_cmd_bootloader_command_t> bootloader_command(m, "JerryCANBootloaderCmd");
    bootloader_command.def(py::init<>())
        .def_readwrite("type", &jerrycan_cmd_bootloader_command_t::type)
    ;

    py::enum_<jerrycan_bootloader_subcmd_t>(bootloader_command, "SubCommand")
        .value("VERSION", JERRYCAN_BOOTLOADER_SUBCMD_VERSION)
        .value("START", JERRYCAN_BOOTLOADER_SUBCMD_START)
        .value("END", JERRYCAN_BOOTLOADER_SUBCMD_END)
        .value("REBOOT", JERRYCAN_BOOTLOADER_SUBCMD_REBOOT)
        .value("FINALIZE", JERRYCAN_BOOTLOADER_SUBCMD_FINALIZE)
        .value("ACK", JERRYCAN_BOOTLOADER_SUBCMD_ACK)
        .value("NACK", JERRYCAN_BOOTLOADER_SUBCMD_NACK)
        .export_values()
    ;

    py::class_<jerrycan_cmd_bootloader_response_t>(m, "BootloaderResponse")
        .def(py::init<>())
        .def_readwrite("type", &jerrycan_cmd_bootloader_response_t::type)
        .def_readwrite("version", &jerrycan_cmd_bootloader_response_t::version)
        .def_readwrite("status", &jerrycan_cmd_bootloader_response_t::status)
    ;

    py::class_<jerrycan_cmd_bootloader_version_t>(m, "BootloaderVersion")
        .def(py::init<>())
        .def_property("running_version_major",
            [](const jerrycan_cmd_bootloader_version_t &a) { return a.running_version_major; },
            [](jerrycan_cmd_bootloader_version_t &a, const uint8_t v) { a.running_version_major = v; })
        .def_property("running_version_minor",
            [](const jerrycan_cmd_bootloader_version_t &a) { return a.running_version_minor; },
            [](jerrycan_cmd_bootloader_version_t &a, const uint8_t v) { a.running_version_minor = v; })
        .def_property("running_version_patch",
            [](const jerrycan_cmd_bootloader_version_t &a) { return a.running_version_patch; },
            [](jerrycan_cmd_bootloader_version_t &a, const uint8_t v) { a.running_version_patch = v; })
       .def_property("slot1_version_major",
            [](const jerrycan_cmd_bootloader_version_t &a) { return a.slot1_version_major; },
            [](jerrycan_cmd_bootloader_version_t &a, const uint8_t v) { a.slot1_version_major = v; })
       .def_property("slot1_version_minor",
            [](const jerrycan_cmd_bootloader_version_t &a) { return a.slot1_version_minor; },
            [](jerrycan_cmd_bootloader_version_t &a, const uint8_t v) { a.slot1_version_minor = v; })
       .def_property("slot1_version_patch",
            [](const jerrycan_cmd_bootloader_version_t &a) { return a.slot1_version_patch; },
            [](jerrycan_cmd_bootloader_version_t &a, const uint8_t v) { a.slot1_version_patch = v; })
    ;

    py::class_<jerrycan_cmd_bootloader_data_t>(m, "BootloaderData")
        .def(py::init<>())
        .def_property("data",
            [](const jerrycan_cmd_bootloader_data_t &a) { return a.data; },
            [](jerrycan_cmd_bootloader_data_t &a, const uint8_t v[sizeof(jerrycan_cmd_bootloader_data_t)]) { std::memcpy(a.data, v, sizeof(jerrycan_cmd_bootloader_data_t)); })
    ;

    py::class_<jerrycan_cmd_fixed_xyz>(m, "FixedXyzCommand")
        .def(py::init<>())
    ;

    py::class_<jerrycan_cmd_delay_t>(m, "DelayCommand")
        .def(py::init<>())
        .def_readwrite("delay", &jerrycan_cmd_delay_t::delay)
    ;

    py::class_<jerrycan_rsp_ack_t>(m, "Acknowledge")
        .def(py::init<>())
        .def_readwrite("error", &jerrycan_rsp_ack_t::error)
    ;


    py::enum_<jerrycan_cmd_type_t>(m, "JerryCANCmdType")
        .value("ESTOP", JERRYCAN_CMD_ESTOP)
        .value("HEARTBEAT", JERRYCAN_CMD_HEARTBEAT)
        .value("STATUS", JERRYCAN_CMD_STATUS)
        .value("STEPPER_MOVE", JERRYCAN_CMD_STEPPER_MOVE)
        .value("SERVO_MOVE", JERRYCAN_CMD_SERVO_MOVE)
        .value("STEPPER_HOME", JERRYCAN_CMD_STEPPER_HOME)
        .value("CFG_WRITE", JERRYCAN_CMD_CFG_WRITE)
        .value("CFG_READ", JERRYCAN_CMD_CFG_READ)
        .value("CFG_RESPONSE", JERRYCAN_CMD_CFG_RESPONSE)
        .value("STEPPER_STATUS", JERRYCAN_CMD_STEPPER_STATUS)
        .value("SERVO_STATUS", JERRYCAN_CMD_SERVO_STATUS)
        .value("PRESSURE_READ", JERRYCAN_CMD_PRESSURE_READ)
        .value("TEMP_HUM_READ", JERRYCAN_CMD_TEMP_HUM_READ)
        .value("GPIO_READ", JERRYCAN_CMD_GPIO_READ)
        .value("GPIO_WRITE", JERRYCAN_CMD_GPIO_WRITE)
        .value("TONE", JERRYCAN_CMD_TONE)
        .value("ANALOG_OUT", JERRYCAN_CMD_ANALOG_OUT)
        .value("LOAD_CELL_READ", JERRYCAN_CMD_LOAD_CELL_READ)
        .value("LOAD_CELL_TARE", JERRYCAN_CMD_LOAD_CELL_TARE)
        .value("RGB_LED", JERRYCAN_CMD_RGB_LED)
        .value("DOOR_SENSOR", JERRYCAN_CMD_DOOR_SENSOR)
        .value("AUDIO_MAGNITUDE_DATA_BEGIN", JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_BEGIN)
        .value("AUDIO_MAGNITUDE_DATA_CONT", JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_CONT)
        .value("AUDIO_MAGNITUDE_DATA_END", JERRYCAN_CMD_AUDIO_MAGNITUDE_DATA_END)
        .value("BOOTLOADER_COMMAND", JERRYCAN_CMD_BOOTLOADER_COMMAND)
        .value("BOOTLOADER_RESPONSE", JERRYCAN_CMD_BOOTLOADER_RESPONSE)
        .value("BOOTLOADER_DATA", JERRYCAN_CMD_BOOTLOADER_DATA)
        .value("DELAY", JERRYCAN_CMD_DELAY)
        .value("FIXED_XYZ", JERRYCAN_CMD_FIXED_XYZ)
        .value("ACKNOWLEDGE", JERRYCAN_RSP_ACK)

        .export_values()
    ;
}
/* clang-format on */
