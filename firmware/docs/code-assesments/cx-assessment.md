# Firmware Code Assessment

Assessment target: repository commit `072bc10`.

## Scope and criteria

This assessment covers runtime code and active board configurations under `firmware/`. It follows the firmware's direct dependency into `software/libjerrycan/inc/jerrycan_types.h` to establish the on-wire contract, but does not assess the rest of `software/`, `hardware/`, `docker/`, or other repository areas. Generated build output was used only to confirm active configuration where useful.

The findings below are concrete paths to incorrect behavior in the current magnet- or pellet-module firmware. Shell-only behavior, cosmetic defects, unused code, documentation-only mistakes, malformed-frame hardening, and issues that would require a future caller or future configuration have been excluded. Per the assessment assumptions, every CAN command is considered a live interface.

Findings are ordered approximately by impact within each category. “Critical” is reserved for loss of a safety function, unsafe actuator output, or a persistent device/subsystem outage.

## Critical Issues

### 1. The CAN emergency-stop command is silently ignored, and the existing stop helper is unsafe

**Affected:** Magnet and pellet modules.

`JERRYCAN_CMD_ESTOP` is part of the shared protocol and has a payload-size entry (`software/libjerrycan/inc/jerrycan_types.h:28-31`, `firmware/lib/jerrycan/jerrycan.c:41-46`), but no receive callback is registered for it. JerryCAN dispatch only invokes matching registered callbacks (`firmware/lib/jerrycan/jerrycan.c:174-183`), so an e-stop frame is accepted and discarded without stopping an actuator or returning an acknowledgement.

The otherwise-unused implementation cannot simply be connected to the command. `motors_all_stop()` iterates the servo array but passes `stepper_motors[i]` to `servo_motor_stop()` (`firmware/lib/motor_motion/motor_motion.c:35-42`), and `set_all_e_stop_flags()` likewise iterates servos but writes `stepper_contexts[i]` (`firmware/lib/motor_motion/motor_motion_workq.c:911-918`). On the magnet module, which has servos but no steppers, those are out-of-bounds accesses. On the pellet module they stop and flag the wrong devices.

**Result:** A client can issue the defined emergency stop while motors continue moving. Wiring the current helper into a callback without correcting both indexing defects can instead fault the magnet firmware and still fail to stop its servos.

### 2. Servo output can go outside both ends of the configured PWM range

**Affected:** Magnet and pellet modules.

There are two independent, currently reachable causes:

- Servo initialization loads a raw compare value of 1,000 timer counts (`firmware/drivers/motor/servo.c:50-57`, `:96-98`). The active servo timers run at 2 MHz, so this is a 500 microsecond pulse, while the default configured minimum is 1,000 microseconds (`firmware/lib/motor_motion/motor_motion_workq.c:32-39`). `SERVO_ATTACH` merely enables the counter (`firmware/lib/jerrycan/modules/servo.c:97-102`); neither attachment nor settings restoration converts the believed/persisted angle to a safe compare value first.
- Movement conversion divides the configured PWM span by a fixed 120 degrees (`firmware/lib/motor_motion/motor_math.c:20`, `:293-307`) instead of by `max_angle - min_angle`. The firmware's own default range is 0–180 degrees. With the default 1,000–2,000 microsecond calibration, a move to the configured 180-degree maximum therefore generates 2,500 microseconds, 500 microseconds beyond the configured endpoint. CAN configuration can also set a different angular range (`firmware/lib/jerrycan/modules/servo.c:137-147`), and every range other than exactly 120 degrees is mapped incorrectly.

**Result:** Merely attaching a servo after boot can command a pulse below its configured minimum, and an ordinary move to the configured maximum can command a pulse above its maximum. Both can drive the horn into a mechanical stop or otherwise produce unsafe physical motion.

### 3. A positional limit-switch stop can corrupt the stepper queue and wedge later moves

**Affected:** Pellet module.

`stepper_move_to_position()` generates and queues as many as two 1,024-entry blocks before returning (`firmware/lib/motor_motion/motor_motion_workq.c:801-830`). The driver removes the first block to configure DMA, leaving the second in the message queue (`firmware/drivers/motor/motor_common.c:62-83`, `:123-140`). If the positional limit switch fires while that queued block remains, the callback calls `stepper_motor_stop()` (`firmware/lib/motor_motion/motor_motion_workq.c:308-325`). That stops DMA and disables the timer channel, but it does not purge the message queue (`firmware/lib/motor_motion/motor_motion.c:22-27`, `firmware/drivers/motor/stepper.c:86-92`).

For a long movement this is normally the steady state rather than a narrow startup window. When DMA completes a block, its callback first removes and reloads the pending block, then schedules calculation of the replacement (`firmware/drivers/motor/motor_common.c:14-39`, `firmware/lib/motor_motion/motor_motion_workq.c:274-286`, `:448-480`). Except for the calculation interval, another block is therefore waiting in the message queue throughout the movement.

The next movement is appended behind the stale entry. Because `ll_motor_queue_data()` only starts idle DMA when the queue contains exactly one entry, the idle motor is not restarted once the queue already contains the stale block (`firmware/drivers/motor/motor_common.c:132-140`). New planning also reuses the same two backing buffers, so stale queue entries can refer to memory overwritten for the new move. Homing does not recover the motor: `stepper_home()` submits through the same queue and therefore also appends behind the stale entry without restarting DMA (`firmware/lib/motor_motion/motor_motion_workq.c:870-892`).

At the same time, `last_position_generated` was advanced while planning rather than while pulses executed (`firmware/lib/motor_motion/motor_math.c:386-432`). The limit-stop path does not correct it, and the periodic completion path treats `MOTION_DONE` as a successful completion (`firmware/lib/jerrycan/modules/stepper.c:159-170`).

**Result:** A normal positional limit event during a sufficiently long move can falsely acknowledge the requested endpoint, corrupt the logical position, and leave that motor unable to execute later accepted commands until its queue/state is reinitialized (normally by reboot).

### 4. CAN transmission can block the only command-processing loop forever

**Affected:** Magnet and pellet modules.

Both applications do all JerryCAN receive dispatch from `while (true) { jerrycan_run(K_FOREVER); }` (`firmware/magnet_module/src/main.c:10-13`, `firmware/pellet_module/src/main.c:10-13`). Before processing receive messages, `jerrycan_run()` drains its transmit queue using synchronous `can_send(..., K_FOREVER, NULL, NULL)` (`firmware/lib/jerrycan/jerrycan.c:135-165`). With a null callback, the call waits for transmission to complete; with no other active node acknowledging frames, a wiring fault, or a bus-off condition that does not complete the send, the wait has no bound.

**Result:** One periodic status/sensor transmission can permanently stop all subsequent CAN command handling, including actuator commands. Timers can continue filling the software TX queue while the main loop remains stuck.

### 5. NAU7802 readiness polling can hang boot or the system workqueue indefinitely

**Affected:** Pellet module.

The load-cell driver polls the NAU7802 power-ready bit until it becomes set and the calibration-active bit until it clears, with no timeout, delay, or iteration bound (`firmware/drivers/load_cell/nau7802.c:59-65`, `:348-359`). An I2C error exits, but a responsive chip that remains in the wrong state does not. The driver itself records that calibration can leave the device non-functional (`firmware/drivers/load_cell/load_cell.c:115-124`).

The power sequence runs during device initialization, so the first loop can prevent boot from reaching the application. The calibration loop is also reached by the live `LOAD_CELL_TARE` command through work submitted to Zephyr's system workqueue (`firmware/lib/jerrycan/modules/load_cell.c:79-99`, `firmware/drivers/load_cell/load_cell.c:97-113`, `:151-158`). The recovery handler runs the power sequence on that same queue.

**Result:** A stuck-ready condition can prevent the firmware from booting. A stuck calibration or recovery after boot permanently occupies the shared system workqueue, stopping load-cell recovery and other work items that use it.

## Issues

### 1. The no-op check rejects ordinary small stepper moves

**Affected:** Pellet module.

Stepper positions are measured in revolutions. The motion generator defines one microstep as `1 / (microsteps * steps_per_revolution)` revolutions (`firmware/lib/motor_motion/motor_math.c:382-403`), but `stepper_move_to_position()` rejects any delta smaller than `1 / microsteps` revolutions (`firmware/lib/motor_motion/motor_motion_workq.c:769-772`). The threshold is too large by the configured `steps_per_revolution` factor.

With the source defaults of one microstep and 48 steps/revolution, every move under one complete revolution is rejected as “the same as current position.” With eight microsteps it still rejects moves under 0.125 revolution, rather than only moves below one physical microstep.

**Result:** Valid absolute and relative move commands return `-EAGAIN` and do not move, including many normal positioning corrections.

### 2. Servo dead-band processing overshoots and compresses trajectories

**Affected:** Magnet and pellet modules.

The servo planner samples a mathematical profile at 20 ms intervals, but it only emits a table entry when the PWM changes by at least four counts (`firmware/lib/motor_motion/motor_math.c:310-369`). Time advances even when an entry is omitted. DMA still consumes each emitted entry for one 20 ms PWM frame, so omitted samples compress the motion in time rather than holding the prior output for the skipped interval.

The dead-band accumulator also sums multiple deltas measured from the same unchanged `last_pwm`, then adds that sum to an already absolute PWM value (`motor_math.c:349-368`). This double-counts the intermediate changes and commands an overshoot whenever several sub-threshold changes precede an emitted value.

There is also a conditional early-stop path. If the rounded PWM is unchanged for 21 samples, the generator emits one last value and stops (`:353-358`). With the active 0.5 microsecond timer increment and 1,000 microsecond PWM span divided over 120 degrees, one timer count is about 0.06 degrees. Twenty-one unchanged samples therefore require less than roughly one count of planned movement over 420 ms. This is principally reachable during the ease-in of a sufficiently gentle acceleration command, where the motion initially grows very slowly; it is not expected in the middle of an ordinary-speed profile.

**Result:** A command can execute faster than requested, the physical PWM can overshoot the planner's believed position, and sufficiently gentle moves can terminate near their starting position while still being acknowledged as complete.

### 3. Long servo movements are silently truncated after the first asynchronous refill

**Affected:** Magnet and pellet modules.

The servo refill handler compares its generated result against the wrong buffer capacity. A servo result is capped at `SERVO_BUFFER_SIZE` (256), but the handler tests whether it is smaller than `STEPPER_BUFFER_SIZE` (1,024) (`firmware/lib/motor_motion/motor_motion_workq.c:403-423`, `firmware/lib/motor_motion/motor_motion_workq.h:12-14`). The condition is therefore true on every asynchronous refill, even when the full servo buffer was generated and more profile remains.

`servo_move_to_position()` initially queues as many as two 256-entry buffers, and the first completion can cause one asynchronous replacement to be queued (`firmware/lib/motor_motion/motor_motion_workq.c:658-680`). The incorrect comparison then marks calculation complete, preventing any further refill. At one PWM entry per 20 ms frame, a full three-buffer movement is capped at 768 frames, or approximately 15.36 seconds.

**Result:** Any valid low-speed profile requiring more than those three buffers stops before its target. Queue-empty handling records the truncated planned position and sends a successful completion acknowledgement.

### 4. Concurrent stepper moves or homes lose all but one completion acknowledgement

**Affected:** Pellet module.

The stepper module keeps one global `moving_state` for all three motors (`firmware/lib/jerrycan/modules/stepper.c:55-59`). Each accepted single move/home sets that global state, and the first completed motor clears it (`:61-101`, `:151-170`, `:311-328`). Concurrent moves on different motors therefore acknowledge only the first completion; later completed contexts are reset to idle without an acknowledgement.

**Result:** When different axes move or home concurrently, a client can wait forever for every completion after the first one observed by the shared state machine.

### 5. A rejected busy-motor command overwrites the active command's UUID

**Affected:** Pellet stepper commands; magnet and pellet servo commands.

Both the stepper and servo handlers overwrite the motor context's UUID before attempting to start the new movement (`firmware/lib/jerrycan/modules/stepper.c:71-89`, `firmware/lib/jerrycan/modules/servo.c:58-75`). If command A is active and command B targets the same busy motor, B replaces A's UUID before the move function returns `-EBUSY`. The dispatcher immediately sends `ack(B, -EBUSY)`. When A later finishes, completion reads the overwritten context and sends `ack(B, 0)`.

**Result:** The accepted command A is never acknowledged, while rejected command B receives both failure and later success for work it never performed.

### 6. Load-cell tare returns a false error before the tare has executed

**Affected:** Pellet module.

`ll_load_cell_tare()` returns the raw normalized work-submission result, for which 1 means a newly queued work item (`firmware/drivers/load_cell/load_cell.c:34-42`, `:151-158`). The CAN handler returns that positive value unchanged (`firmware/lib/jerrycan/modules/load_cell.c:79-99`), and the JerryCAN dispatcher places any ordinary callback return into the acknowledgement (`firmware/lib/jerrycan/jerrycan.c:174-181`). A normal tare submission is therefore acknowledged immediately with error value `1`, rather than success after completion.

The actual calibration happens asynchronously, and its failure paths only log and return (`firmware/drivers/load_cell/load_cell.c:97-113`). No later success or failure acknowledgement is sent.

**Result:** Successful tare requests are reported as failures, actual failures cannot be correlated to the command, and any client waiting for a completion acknowledgement receives incorrect state.

### 7. Stepper microstep configuration can be persisted and acknowledged although the driver rejected it

**Affected:** Pellet module.

`stepper_set_parameters()` updates the motion context, calls `adi_tmc2209_set_microstep()`, ignores its return, persists the new value, and returns success (`firmware/lib/motor_motion/motor_motion_workq.c:693-726`). The TMC2209 function rejects values outside 1, 2, 4, ... 256 and can also fail to read the device (`firmware/drivers/motor/adi_tmc2209.c:410-457`). Even its write helper discards the result of the single-wire UART transaction and always returns zero (`:143-163`). The boot-time reapplication also ignores the result (`firmware/lib/motor_motion/motor_motion_workq.c:536-545`).

The motion planner immediately uses the new software value to calculate travel (`motor_motion_workq.c:790-794`), even if the physical driver's setting did not change.

**Result:** Subsequent moves can travel by the wrong scale factor while configuration reads and acknowledgements claim the requested setting is active. The mismatch survives reboot.

### 8. Tone command validation and completion tracking are broken

**Affected:** Pellet module.

The tone driver detects frequencies outside its supported range but only logs; it continues enabling the amplifier/DMA and calculating the timer reload (`firmware/drivers/tone_generator/tone_generator.c:295-325`). A frequency of zero reaches an integer division by zero in the STM32 timer macro. On this target that need not trap; it can instead produce a wrapped reload value and hold the enabled audio path at a near-DC/invalid sample rate until the duration expires. Other unsupported values are likewise accepted rather than rejected.

The CAN layer uses `uuid != 0` as its “ack pending” flag even though `uuid_t` is the full `uint8_t` domain with no reserved-zero rule (`software/libjerrycan/inc/jerrycan_types.h:21`, `firmware/lib/jerrycan/modules/tone.c:65-84`). UUID zero is never acknowledged. A new tone also overwrites the UUID of a tone that the driver explicitly aborts, orphaning the first acknowledgement (`firmware/lib/jerrycan/modules/tone.c:94-128`). On a driver-start error, the immediate status transmission can send success for the stored UUID before the dispatcher sends the real failure.

**Result:** Valid client correlation IDs can hang, overlapping tones lose completions, failures can produce contradictory acknowledgements, and frequency zero drives the hardware with an invalid timer configuration.

### 9. Firmware status reporting is absent or knowingly false

**Affected:** Magnet and pellet modules.

The complete construction and transmission of `JERRYCAN_CMD_STATUS` is inside `#if 0`, although its periodic timer is started (`firmware/lib/jerrycan/modules/status.c:38-64`, `:86-90`). Clients therefore never receive the defined aggregate status containing e-stop, limit, and button state (`software/libjerrycan/inc/jerrycan_types.h:78-96`).

The periodic motor-specific messages do transmit, but both stepper and servo `.status` fields are hard-coded to zero (`firmware/lib/jerrycan/modules/stepper.c:336-351`, `firmware/lib/jerrycan/modules/servo.c:217-223`).

**Result:** A live protocol feature produces no frames, while the available motor status frames report a healthy/zero state regardless of actual motor state or error.

### 10. Stepper position telemetry reports planned future position as current position

**Affected:** Pellet module.

Stepper status reports `context->context.last_position_generated` (`firmware/lib/jerrycan/modules/stepper.c:336-351`). That field advances as timing blocks are generated, before those blocks are transferred by DMA and before their steps physically execute (`firmware/lib/motor_motion/motor_math.c:386-432`). The move path pre-generates up to two blocks (`firmware/lib/motor_motion/motor_motion_workq.c:807-830`).

**Result:** Short moves can report their final target almost immediately after acceptance while the motor is still moving. Longer moves report as much as the pre-generated queue ahead of the actual mechanism; limit stops retain the planned value rather than the reached one.

### 11. Bootloader responses can contain uninitialized state or use the wrong response protocol

**Affected:** Native MCUboot builds for both modules.

The version handler declares an uninitialized MCUboot header, ignores `boot_read_bank_header()`'s return, and reports fields from it (`firmware/lib/jerrycan/modules/bootloader.c:53-72`). When slot 1 is empty or invalid, the reported slot version is indeterminate.

The data handler also declares an uninitialized response and initializes only its top-level type and ACK/NACK subtype on every response path (`bootloader.c:185-215`). The seven-byte response always includes the otherwise-uninitialized status union, so `active` and `bytes_written` are stack garbage for successful writes, failed writes, and “data before START.” If page erase fails, the handler returns raw errno without sending a `BOOTLOADER_RESPONSE`; the generic JerryCAN acknowledgement is a different message type than the bootloader data exchange otherwise uses (`:189-200`).

**Result:** Update clients can receive random progress/version data or wait for a bootloader response that never arrives on an erase failure.

### 12. Negative servo minimum angles are silently changed after reboot

**Affected:** Magnet and pellet modules.

CAN configuration accepts and saves servo `min_angle` (`firmware/lib/jerrycan/modules/servo.c:137-147`, `firmware/lib/motor_motion/motor_settings.c:243-246`). Settings restoration reads it with validation that replaces every value `<= 0` with zero (`firmware/lib/motor_motion/motor_settings.c:72-85`, `:188-203`). Although the same loader rule is textually applied to `angle_adjustment`, neither active board has a reachable path that gives that field a nonzero value, so that field is not part of this finding.

**Result:** A valid negative angle range works until reboot, then silently changes to a zero-based range. The persisted position can subsequently fail range validation, and physical angle-to-PWM mapping changes without any new CAN configuration command.

### 13. Negative temperatures are transmitted as plausible zero-degree readings

**Affected:** Magnet module.

The sensor path casts `temperature * 100` to `uint16_t` (`firmware/lib/jerrycan/modules/temperature.c:137-145`). The directly included protocol type is also unsigned even though its own stated sensor range extends to -40 degrees C (`software/libjerrycan/inc/jerrycan_types.h:185-207`). There is no signed or offset decoding convention. In C, an out-of-range floating-to-unsigned conversion is undefined; for the generated magnet-module object in this checkout, the compiler emits `vcvt.u32.f32`, which saturates a negative input to zero.

**Result:** In the assessed build, any sub-zero measurement is transmitted as `0` and decoded as a plausible 0.00 degrees C reading. Independently of that particular code generation, the unsigned wire type cannot represent the documented negative range.

### 14. Audio spectrum frames contain the wrong frequency bins

**Affected:** Magnet module.

For an N-sample real FFT, the code produces `N/2 + 1` values including DC and Nyquist (`firmware/lib/jerrycan/modules/audio.c:44-45`). `audio_report_data()` says it skips DC and starts its loop counter at one, but it leaves the data pointer at `magnitude[0]` (`:270-288`). With the active 128-sample block and 16 floats per CAN-FD payload, it transmits bins 0 through 63: DC is included, Nyquist bin 64 is omitted, and a consumer that interprets the stream as DC-skipped assigns every magnitude to the next frequency bin.

**Result:** Spectral peaks are reported in the wrong bins, with one intended endpoint missing.

### 15. Overlapping DELAY commands permanently lose the first acknowledgement

**Affected:** Magnet and pellet modules.

The delay implementation has one global UUID and one global timer. Each `JERRYCAN_CMD_DELAY` overwrites the UUID and restarts the timer (`firmware/lib/jerrycan/modules/status.c:66-79`).

**Result:** If a second delay arrives while the first is pending, only the second UUID is acknowledged. A client awaiting the first live command waits forever.

### 16. GPIO callback registration emits unrelated GPIO frames on the pellet board

**Affected:** Pellet module.

The generic-GPIO driver ORs the physical pin numbers from all of its GPIO ports into one mask, then registers the same callback object on every port (`firmware/drivers/gpio/generic_gpios.c:543-560`). The pellet generic GPIOs span GPIOC pins 4/5 and GPIOB pins 0/1/11-14 (`firmware/boards/arm/cerebellumlab_pellet_module/cerebellumlab_pellet_module.dts:61-81`), so the callback registered on GPIOC also claims pins 11-14. Pellet door sensors use GPIOC 13-15 and have their own active interrupts.

**Result:** Door activity on GPIOC 13 or 14 also matches the generic callback and produces an unsolicited generic `GPIO_READ` transmission.

### 17. A GPIO read failure during startup can select an indeterminate CAN node address

**Affected:** Magnet and pellet modules.

`get_can_node_id()` leaves `device_type` and `node_id` uninitialized, ignores both GPIO-read returns, and immediately constructs the CAN ID from those locals (`firmware/lib/jerrycan/jerrycan.c:25-38`). The GPIO helper returns without writing its output when an individual pin read fails (`firmware/drivers/gpio/generic_gpios.c:192-211`).

**Result:** A startup GPIO read error can install CAN filters and transmit frames under a stack-derived address, making the module disappear from its expected address rather than fail initialization deterministically.

## Verification notes

This is a source/configuration assessment rather than hardware-in-the-loop validation. The active magnet and pellet board definitions and existing generated configurations were inspected to distinguish live paths from code that is not built for either product. The existing magnet-module object was disassembled to confirm the current ARM conversion used for negative temperature values. A fresh Zephyr build could not be used as a verification gate in this checkout because the available build caches refer to a different checkout path and `west` is not installed in the current host environment. That limitation does not change the source-level control-flow and arithmetic findings above.
