zephyr_library()

zephyr_include_directories(. ../../../software/libjerrycan/inc)

zephyr_library_sources(
	       jerrycan.c
	       modules/heartbeat.c
	       modules/status.c
	       modules/gpio.c
		   modules/uptime.c
)

# If this board has the motor motion library enabled, include the stepper and servo files
# to include the appropriate CAN message handlers
if (CONFIG_LIB_MOTOR_MOTION)
	zephyr_library_sources(
		          modules/estop.c
	    )

	zephyr_library_sources_ifdef(CONFIG_DT_HAS_LL_SERVO_ENABLED
		          modules/servo.c
	    )

	zephyr_library_sources_ifdef(CONFIG_DT_HAS_LL_STEPPER_ENABLED
		          modules/stepper.c
	    )
endif ()

# If this board has a temperature sensor...
zephyr_library_sources_ifdef(CONFIG_LIB_JERRYCAN_TEMPERATURE_SENSOR
	       modules/temperature.c
)

# If this board has a tone generator...
zephyr_library_sources_ifdef(CONFIG_DT_HAS_LL_TONE_GENERATOR_ENABLED
	       modules/tone.c
)

# If this board has a pressure sensor...
zephyr_library_sources_ifdef(CONFIG_DT_HAS_LL_PRESSURE_SENSOR_ENABLED
	       modules/pressure.c
)

# If this board has a load cell...
zephyr_library_sources_ifdef(CONFIG_DT_HAS_LL_LOAD_CELL_ENABLED
	       modules/load_cell.c
)

# If this board has an analog output...
zephyr_library_sources_ifdef(CONFIG_DT_HAS_LL_ANALOG_OUT_ENABLED
	       modules/analog.c
)

# If this board has doors
zephyr_library_sources_ifdef(CONFIG_DT_HAS_GPIO_KEYS_ENABLED
	       modules/door_sensor.c
)

# If this board has a microphone...
zephyr_library_sources_ifdef(CONFIG_DT_HAS_LL_MICROPHONE_ENABLED
	       modules/audio.c
)

# If this board and an RGB LED
zephyr_library_sources_ifdef(CONFIG_LIB_JERRYCAN_RGB_LED
	       modules/rgb_led.c
)

zephyr_library_sources_ifdef(CONFIG_LIB_JERRYCAN_SHELL
	       jerrycan_shell.c
)

zephyr_library_sources_ifdef(CONFIG_BOOTLOADER_MCUBOOT
        modules/bootloader.c
)
