# Firmware Code Assessment

_Scope: `firmware/`, plus `software/libjerrycan/inc/jerrycan_types.h`, which the firmware includes directly.
Generated build trees under `firmware/*/build/`, the Zephyr checkout under `deps/`, and `objdump` on shipped
object files were used only as evidence about the current build._

_Revision 2. Revised against an independent evaluation (`cc-assessment-eval.md`); see "Revision Notes" at the
end for what changed and for the two evaluation points I checked and did not accept._

## Criteria

Every item is reachable in code that is built for at least one active board today. Items reachable only
through a shell command, only after a future change, or only from an invalid client argument are excluded from
the numbered findings; a few of the last kind are recorded in a clearly-marked appendix and are not counted.

**Critical** is reserved for loss of a safety function, unsafe physical output, or a persistent device or
subsystem outage. Protocol correlation errors, rejected commands and wrong telemetry are Issues, however
disruptive, unless they produce one of those three effects.

Board configuration determines reachability for most findings:

| | steppers | servos | other |
|---|---|---|---|
| `cerebellumlab_magnet_module` | 0 | 2 | microphone/FFT, pressure, SHT3xD temp/humidity, **NAU7802 load cell** |
| `cerebellumlab_pellet_module` | 3 | 3 | tone generator, analog out, door sensors, RGB LED |

---

## Critical Issues

### 1. The e-stop command does nothing at all

**Affected:** both modules.

`JERRYCAN_CMD_ESTOP` is defined in the protocol header the firmware includes
(`jerrycan_types.h:29`) and has a payload-size entry (`lib/jerrycan/jerrycan.c:44`), but no receive callback is
registered for it anywhere. `jerrycan_run()` only invokes callbacks whose `filter_msg_type` matches
(`jerrycan.c:174-183`), so the frame is decoded and discarded. `trigger_e_stop()`, `motors_all_stop()` and
`set_all_e_stop_flags()` (`lib/motor_motion/motor_motion.c:35-49`,
`lib/motor_motion/motor_motion_workq.c:911`) have no callers in the tree.

**A host e-stop leaves every stepper and servo running.**

The dead implementation it would call cannot simply be wired up. Two copy-paste indexing bugs:

```c
// lib/motor_motion/motor_motion.c:40-42  — iterates servos, indexes steppers
for (size_t i = 0; i < ARRAY_SIZE(servo_motors); i++) {
    servo_motor_stop(stepper_motors[i]);
}

// lib/motor_motion/motor_motion_workq.c:916-918  — same mistake
for (size_t i = 0; i < ARRAY_SIZE(servo_contexts); i++) {
    atomic_flag_test_and_set(&stepper_contexts[i].e_stop_triggered);
}
```

On the magnet module `stepper_motors[]` and `stepper_contexts[]` are zero-length arrays (0 steppers, 2
servos), so both loops read and write past the end of an empty array. On the pellet module (3 of each) they
are in bounds but stop and flag the wrong devices — `servo_motor_stop()` would disable a *stepper's* timer
counter and leave all three servos running.

### 2. A `TONE` command with frequency 0 halts the module permanently

**Affected:** pellet module.

`ll_tone_generator_play_tone()` checks the supported frequency range and then falls through without returning:

```c
// drivers/tone_generator/tone_generator.c:304-307
if (frequency_hz < TONE_GENERATOR_MIN_FREQUENCY || frequency_hz > TONE_GENERATOR_MAX_FREQUENCY) {
    LOG_ERR("Invalid frequency <%d> ...", ...);
}
/* no return — enables the amplifier, starts DMA, then: */
LL_TIM_SetAutoReload(cfg->sample_rate_timer,
    __LL_TIM_CALC_ARR(SystemCoreClock, 0x0, frequency_hz * (sizeof(sine_wave) / sizeof(sine_wave[0]))));
```

The HAL macro is `((TIMCLK/(PSC+1)) >= FREQ) ? ((TIMCLK/(FREQ*(PSC+1))) - 1) : 0`
(`deps/modules/hal/stm32/stm32cube/stm32g4xx/drivers/include/stm32g4xx_ll_tim.h:1921-1922`). With
`frequency_hz == 0` the unsigned comparison passes and the divide executes. Both operands are runtime values,
so this is a real division, confirmed in the shipped object:

```text
$ objdump -d .../tone_generator.c.obj
00000000 <ll_tone_generator_play_tone>:
  f0:   udiv    r3, r3, r1
```

Zephyr enables divide-by-zero trapping unconditionally on ARMv7-M — `arch_kernel_init()` calls
`z_arm_fault_init()` (`deps/zephyr/arch/arm/include/cortex_m/kernel_arch_func.h:43`), which sets
`SCB_CCR_DIV_0_TRP_Msk` (`deps/zephyr/arch/arm/core/cortex_m/fault.c:1171`). `CONFIG_ARMV7_M_ARMV8_M_MAINLINE`
is set for this target. Neither application overrides `k_sys_fatal_error_handler()`, so the weak default runs
`arch_system_halt()` (`deps/zephyr/kernel/fatal.c:37-46`).

**Result:** `ToneWrite(..., frequency = 0, ...)` raises a UsageFault and the module halts — no CAN, no motors,
no recovery short of a power cycle, with any in-flight motion abandoned wherever it was. `frequency` is an
unconstrained `uint16_t` in the protocol, and 0 is the obvious value for a client trying to silence the tone.
The same missing `return` also lets any other unsupported frequency through to the hardware.

### 3. The NAU7802 polling loops have no timeout, and one is reachable from a CAN command

**Affected:** magnet module (the load cell is at `cerebellumlab_magnet_module.dts:200`).

`drivers/load_cell/nau7802.c` has two unbounded spin-on-I²C loops:

```c
// :60-65  (nau7802_power_sequence) — wait for power-up
do {
    ret = nau7802_read(i2c, NAU7802_PU_CTRL, (nau7802_reg_t *)&pu_ctrl);
    if (ret != 0) { return ret; }
} while (!pu_ctrl.pur);

// :352-359 (nau7802_calibrate) — wait for calibration to finish
do {
    ret = nau7802_read(i2c, NAU7802_CTRL2, (nau7802_reg_t *)&ctrl2);
    if (ret != 0) { return ret; }
} while (ctrl2.cals);
```

Both exit only on an I²C transport error. A chip that ACKs its address but never asserts `PUR`, or never
clears `CALS`, spins forever with no delay and no iteration cap. The driver's own comment records that the
calibration sequence does sometimes leave the part "in a non-functional state"
(`drivers/load_cell/load_cell.c:120-123`).

Three reach it:

- `JERRYCAN_CMD_LOAD_CELL_TARE` → `ll_load_cell_tare()` → `tare_i2c_work` → `nau7802_calibrate()`, **on the
  system workqueue**. This is the only CAN-triggered path.
- The DRDY watchdog (`ll_load_cell_drdy_expired` timer, not CAN) → `drdy_failed_work` →
  `nau7802_power_sequence()`, also on the system workqueue. That handler additionally calls
  `k_sleep(K_MSEC(10))` directly (`load_cell.c:129`) plus another 10 ms inside `nau7802_power_sequence`,
  blocking the shared queue for ≥20 ms on every recovery attempt even when the part is healthy.
- `ll_load_cell_nau7802_init()` runs at `POST_KERNEL`, so a dead part hangs boot before `main()`.

Because the load cell and the SHT3xD temperature sensor are both on the magnet module
(`cerebellumlab_magnet_module.dts:188-192`, `:200-211`), a wedged system workqueue takes
`jerrycan_temperature_sensor_tx_work` (`lib/jerrycan/modules/temperature.c:81`) with it, along with the load
cell's own DRDY read work — so the recovery mechanism is inside the thing that is stuck.

### 4. `can_send()` waits forever on completion, blocking all receive processing

**Affected:** both modules.

`main()` is `while (true) { jerrycan_run(K_FOREVER); }` on both boards, and every RX callback runs in that
thread. `jerrycan_run()` drains its TX queue before touching RX, using a synchronous send:

```c
// lib/jerrycan/jerrycan.c:157
ret = can_send(can_dev, &frame, K_FOREVER, NULL, NULL);
```

The `timeout` argument does **not** bound this call. With a `NULL` callback, Zephyr's shim allocates a
completion semaphore, passes `timeout` to the driver only for acquiring a free TX mailbox, and then waits
unconditionally:

```c
// deps/zephyr/drivers/can/can_common.c:59-72
err = api->send(dev, frame, timeout, can_tx_default_cb, &ctx);
if (err != 0) { return err; }
k_sem_take(&ctx.done, K_FOREVER);
```

A frame that is never acknowledged is retransmitted indefinitely and never completes, so the **first**
synchronous send blocks forever — filling the hardware mailboxes is not required. With no other node on the
bus, a wiring fault, or a bus-off that does not complete the transfer, the module stops servicing
`jerrycan_rx_msgq` (10 slots, `jerrycan.c:15`) entirely and is unreachable rather than merely quiet. The
periodic TX timers keep filling the 150-slot software queue behind it.

---

## Issues

### 1. Overlapping stepper commands lose their acknowledgements

**Affected:** pellet module.

`lib/jerrycan/modules/stepper.c` tracks "a move is outstanding" in one **module-global** variable
(`moving_state`, line 59) while the uuid is per motor context. The first completion clears the global:

```c
// :159-172
for (int i = 0; i < STEPPER_COUNT; ++i) {
    if (context && context->motion_mode == MOTION_DONE) {
        context->motion_mode = MOTION_IDLE;
        if (moving_state == MOVING_SINGLE || moving_state == MOVING_COMPLETE) {
            jerrycan_send_ack(context->uuid, 0);
            moving_state = MOVING_NONE;      // global, cleared after the first motor
        }
    }
}
```

**Concurrent moves on different motors.** `StepperMove(motor 0, uuid A)` then `StepperMove(motor 1, uuid B)`:
both set `MOVING_SINGLE` and return `COMMAND_NOT_COMPLETE`. Motor 0 finishes → ack `A`, state → `MOVING_NONE`.
Motor 1 finishes → its `motion_mode` is reset to idle but the global no longer matches, so **`B` is never
acknowledged** and the client blocks forever. `stepper_home_handler` sets the same global (line 321), so
homing three axes concurrently — the obvious startup sequence — acknowledges exactly one.

**`FIXED_XYZ` issued while any stepper is moving.** This is deterministic, not a race.
`stepper_fixed_move()` calls `set_uuid_for_xyz_context()` (`:126-133`), which overwrites the uuid of **all
three** contexts, then sets `moving_state = MOVE_X`. An in-flight move's uuid is destroyed, and its completion
is later consumed with the global in a `MOVE_*`/`MONITOR_*` state, so no ack is sent for it at all. Worse, the
sequence's first step calls `attempt_motor_move(0)` on the still-busy motor; `stepper_move_to_position()`
returns `-EBUSY`, and `attempt_motor_move()` responds by setting that context to `MOTION_DONE` (`:143-146`) —
marking a motor that is physically still running as complete. `stepper_handle_motion_complete()` then resets
it to `MOTION_IDLE`, so when the DMA really does drain, `stepper_motor_event_callback()` finds the context
idle and never raises `MOTION_DONE` again. The X axis is also silently skipped in the XYZ sequence.

### 2. Small stepper moves are rejected as "already at the target"

**Affected:** pellet module.

`lib/motor_motion/motor_motion_workq.c:769`:

```c
if (fabsf(target_position - context->context.last_position_generated) < 1.0f / context->microsteps) {
    LOG_WRN("Target position is the same as current position.");
    return -EAGAIN;
}
```

Stepper positions are **revolutions** (`lib/motor_motion/motor_math.h:15`, `:77`). One microstep is
`1 / (microsteps × steps_per_revolution)` revolutions — the generator uses exactly that
(`motor_math.c:402-403`). The guard compares against `1 / microsteps`, too coarse by a factor of
`steps_per_revolution`.

With the source-initialised values (`STEPPER_DEFAULT_MICRO_STEPS = 1`, `STEPPER_DEFAULT_STEPS_PER_REVOLUTION =
48`) the guard rejects **any commanded move smaller than one full revolution**. With 8 microsteps — the value
the settings loader falls back to when the stored record is unreadable (`motor_settings.c:345`) — it still
rejects anything under 0.125 rev ≈ 45°. The client receives `-EAGAIN` and the motor does not move; nothing in
the log distinguishes this from a genuine no-op.

### 3. Load cell tare reports an error before the tare has run

**Affected:** magnet module.

`drivers/load_cell/load_cell.c:152-158` returns `k_work_submit()`'s raw value:

```c
int ll_load_cell_tare(const struct device *dev) {
    const int ret = k_work_submit(&data->tare_i2c_work);
    return k_work_error_handler(ret);   // returns `error` unchanged
}
```

`jerrycan_load_cell_tare_handler()` (`lib/jerrycan/modules/load_cell.c:79-100`) passes it through and
`jerrycan_run()` puts it in `ack.error`. The contract in `jerrycan_types.h:368` is "0 is OK, negative is a
system errno constant", so the positive returns are not valid values at all:

- **1** — work newly queued. This is the normal case for a tare issued while no tare is pending, and it is
  reported to the host as a failure.
- **2** — work was running and has been requeued. Also an invalid positive.
- **0** — work was already queued. Reported as success, but prematurely: nothing has been calibrated yet.

In every case the acknowledgement precedes the I²C work, and the asynchronous failure paths
(`ll_load_cell_tare_i2c_work_handler`, `:97-113`) only log — so an actual tare failure is never reported to
the requester at all.

### 4. The tone acknowledgement protocol is broken three ways

**Affected:** pellet module.

`lib/jerrycan/modules/tone.c` defers the ack until the tone finishes, using `context->uuid != 0` as the
"ack pending" flag:

```c
// :81-84
if (msg.tone.duration_ms == 0 && context->uuid != 0) {
    jerrycan_send_ack(context->uuid, 0);
    context->uuid = 0;
}
```

- **uuid 0 is never acknowledged.** `uuid_t` is a plain `uint8_t` (`jerrycan_types.h:21`) with no reserved
  value, and `ToneWrite()` takes it straight from the caller. The flag and the value share one variable, so a
  tone issued with uuid 0 completes and the ack is suppressed forever.
- **A failed tone is acknowledged twice, success first.** The handler sets `contexts[idx].uuid = msg->uuid`
  (line 116) *before* calling `ll_tone_generator_play_tone()`. If that fails, the immediate
  `jerrycan_tone_generator_tx()` at line 125 sees `duration_ms == 0` — the duration timer was never started —
  and sends `ack(uuid, 0)`. The handler then returns the non-zero `rc` (line 128) and `jerrycan_run()` sends a
  second ack with the real error, same uuid.
- **An overlapping tone drops the first ack.** A second `TONE` while one is playing overwrites
  `contexts[idx].uuid`. `ll_tone_generator_play_tone()` explicitly supports this — it aborts the running tone
  at `tone_generator.c:295-301` — so it is a supported call sequence, and the first uuid is simply lost.

### 5. Long servo moves are silently truncated (wrong buffer-size constant)

**Affected:** both modules.

`lib/motor_motion/motor_motion_workq.c:417`, inside the **servo** calculation handler:

```c
if (ret < STEPPER_BUFFER_SIZE) {     // should be SERVO_BUFFER_SIZE
    context->motion_calculation_done = true;
}
```

`SERVO_BUFFER_SIZE` is 256 and `STEPPER_BUFFER_SIZE` is 1024 (`motor_motion_workq.h:12-13`), so `ret` — capped
at 256 — is *always* less than the constant tested. `motion_calculation_done` is set on the first asynchronous
refill regardless of whether the profile is finished, and no further calculation work is ever scheduled
(`servo_motor_event_callback`, line 349).

`servo_move_to_position()` preloads two buffers and gets the constant right (line 673), so a move is capped at
`2 × 256 + 256 = 768` PWM frames ≈ 15.4 s of output. A valid request with a low enough velocity or
acceleration needs a longer profile than that, stops short of the commanded angle, is transitioned to
`MOTION_DONE`, and is acknowledged as successful. Position bookkeeping stays self-consistent
(`known_position` follows `last_position_generated`), so the servo also *reports* the truncated position.

### 6. A non-power-of-2 microstep setting desynchronises software and hardware

**Affected:** pellet module.

`lib/motor_motion/motor_motion_workq.c:713-717`:

```c
if (microsteps > 0) {
    context->microsteps = microsteps;
    const ll_motor_cfg_t *stepper_config = dev->config;
    adi_tmc2209_set_microstep(stepper_config->stepper_driver_device, microsteps);
}
```

`adi_tmc2209_set_microstep()` rejects anything outside {1,2,4,…,256} with `-EINVAL`
(`drivers/motor/adi_tmc2209.c:450-452`) and leaves `CHOPCONF.mres` unchanged. The return is discarded,
`context->microsteps` has already been overwritten, `motor_settings_save()` persists it (line 725), and
`stepper_set_parameters()` returns 0 — so the host gets a success ack while the planner's
`min_step = 1/microsteps` no longer matches the driver chip. Every subsequent position is wrong by that ratio,
and the mismatch survives reboot. `StepperCfgWrite` passes the client's value through untouched.

`adi_tmc2209_write()` also returns 0 unconditionally (`adi_tmc2209.c:143-163`), discarding
`write_single_line_uart_and_flush_read()`'s result, so even a valid microstep value that fails on the wire is
reported as applied.

### 7. Bootloader data responses carry garbage, and an erase failure leaves the protocol

**Affected:** both modules (both set `CONFIG_BOOTLOADER_MCUBOOT`).

`jerrycan_bootloader_rx_data_handler()` (`lib/jerrycan/modules/bootloader.c:185-216`) declares
`jerrycan_msg_t resp;` uninitialised and, on **every** path, writes only `resp.type` and
`resp.bootloader_response.type`. The `status.active` / `status.bytes_written` assignments exist solely in the
*command* handler (`:178-179`). `JERRYCAN_CMD_BOOTLOADER_RESPONSE` has a 7-byte payload, all transmitted — so
every response to a `BOOTLOADER_DATA` frame delivers six bytes of stack garbage where the host reads its
progress counter. A host that uses `bytes_written` to track or resume a transfer is reading noise.

Separately, an erase failure abandons the response protocol entirely:

```c
// :197-200
int ret = erase_page(bootloader_ctx.offset);
if (ret) {
    return ret;          // no BOOTLOADER_RESPONSE sent
}
```

Every other exit transmits a `BOOTLOADER_RESPONSE` and returns `SEND_NO_ACKNOWLEDGEMENT`. This one returns a
raw errno, so `jerrycan_run()` emits a generic `JERRYCAN_RSP_ACK` instead. The uuid on that ack is worse than
merely unrelated: `BOOTLOADER_DATA` occupies the full 64-byte payload, so the `msg_len < sizeof(msg.payload)`
guard at `jerrycan.c:170` is false and `msg.uuid` is never refreshed — the static `msg` object
(`jerrycan.c:108`) still holds the uuid of some **earlier** command. The host can therefore see a spurious
success for an unrelated pending request while its DFU loop waits for a `BOOTLOADER_RESPONSE` that never
comes.

`boot_read_bank_header()`'s return is also discarded at `:57`, so a failed read reports an indeterminate
slot-1 version.

### 8. The servo displacement table overshoots and compresses the profile in time

**Affected:** both modules.

`lib/motor_motion/motor_math.c:350-369`:

```c
const uint32_t pwm = degrees_to_pwm_count(context, displacement_now, false);
const int32_t delta_pwm = pwm - last_pwm;
...
} else if (fabs(delta_pwm) >= DEAD_BAND_THRESHOLD) {
    last_pwm = pwm + accumulate;         // absolute value plus a sum of offsets
    table[table_index++] = last_pwm;
    accumulate = 0;
} else {
    accumulate += delta_pwm;
    pwm_change_zero_count = 0;
}
```

`delta_pwm` is measured against `last_pwm`, which does not move while sub-deadband samples are skipped, so
`accumulate` sums *cumulative* offsets from a fixed reference (1 + 2 + 3 = 6 for a monotonic ramp), not
incremental ones. `pwm` is already the correct absolute count for `displacement_now`, so adding `accumulate`
commands the horn past the intended position by that sum. For decreasing motion the unsigned arithmetic
produces the mirror-image negative offset. `context->last_position_generated` is set from the mathematical
`displacement_now` (line 377), so the believed position does not include the overshoot.

Independently: a skipped sample consumes a 20 ms slot of the planned profile but emits no table entry, while
the DMA consumes one entry per 20 ms PWM frame. Any run of deadband-suppressed samples therefore traverses
that segment faster than the commanded velocity profile.

### 9. The periodic `STATUS` message is compiled out

**Affected:** both modules.

`lib/jerrycan/modules/status.c:41-61` — the entire body of `jerrycan_status_tx()` sits inside `#if 0`, behind
a `FIXME: TODO`. The 1 s timer still runs and calls an empty function, so `JERRYCAN_CMD_STATUS` — carrying
`estop_active`, three limit-switch bits and a button bit — is never transmitted by either board.

In the motor status messages that *are* sent, `jerrycan_cmd_stepper_status_t.status` and
`jerrycan_cmd_servo_status_t.status` are hard-coded to 0 (`modules/stepper.c:346`, `modules/servo.c:221`), so
those bytes never carry runtime state either.

### 10. Negative servo angle limits cannot survive a reboot

**Affected:** both modules.

A negative `min_angle` is an ordinary servo configuration, and `ServoCfgWrite` accepts one and persists it
(`modules/servo.c:143` → `servo_set_angle_parameters()` → `motor_settings_save()`). Restoration replaces it:

```c
// lib/motor_motion/motor_settings.c:85, reached from :195 with validate=true, dflt=0
return validate && (value <= 0.0f) ? dflt : value;
```

On the next boot `min_angle` comes back as 0. That feeds `servo_settings_commit()` (`:276-303`), which
validates the stored horn position against the corrupted `[0, max_angle]` window, rejects a legitimately
negative stored position, and falls back to `servo_assume_min_angle_position()`. The result is a believed
position that does not match the hardware until the host rewrites the configuration, and an angle-to-PWM
mapping that has silently changed without any CAN command.

### 11. A second `DELAY` command cancels the first one's acknowledgement

**Affected:** both modules.

`lib/jerrycan/modules/status.c:66-79` keeps one global uuid and one timer:

```c
static struct _delay { uint8_t uuid; } delay_data;

static int status_delay(const jerrycan_msg_t *msg) {
    delay_data.uuid = msg->uuid;
    k_timer_start(&delay_timer, K_MSEC(msg->delay.delay), K_MSEC(0));
    return COMMAND_NOT_COMPLETE;
}
```

Starting an already-running Zephyr timer restarts it, so a `DELAY` issued while another is pending both
overwrites the uuid and resets the expiry. Since `DELAY`'s only purpose is to be an awaited no-op, a client
that pipelines two of them deadlocks on the first.

### 12. Audio magnitude frames drop a bin and contradict their own stated origin

**Affected:** magnet module.

`lib/jerrycan/modules/audio.c:284-288`:

```c
// Skip over DC (assumption, for the moment)
for (int k = 1; k < length; k += ELEMENTS_PER_MESSAGE) {
    audio_transmit_data(magnitude, MIN(length - ELEMENTS_PER_MESSAGE, ELEMENTS_PER_MESSAGE) * sizeof(float));
    magnitude += ELEMENTS_PER_MESSAGE;
}
```

With `block-size = 128` (`cerebellumlab_magnet_module.dts:77`), the FFT produces `128/2 + 1 = 65` magnitudes
and `ELEMENTS_PER_MESSAGE` is 16. `k` takes 1, 17, 33, 49 — four 64-byte `_CONT` frames — while the pointer
starts at `magnitude[0]`. So the stream is bins **0–63**.

What is unambiguous: **the Nyquist bin (64) is never transmitted**, and nothing on the wire tells the receiver
which of the 65 values is missing. What the loop counter starting at 1 is doing is supplying the iteration
count for a DC-skipped stream; the pointer was not advanced to match. Whether this leaves the *host* misaligned
by one bin (1500 Hz at 192 kHz / 128) depends on the origin the host assumes, which cannot be settled from
firmware sources — see "Unresolved" below.

The length argument is also not general: it should be `MIN(length - k, ...)`, since `length` does not change
inside the loop. With the current numbers it evaluates to 16 every iteration and stays in bounds.

### 13. Negative temperatures are transmitted as 0.00 °C

**Affected:** magnet module.

`lib/jerrycan/modules/temperature.c:143`:

```c
.temperature = (uint16_t)(sensor_value_to_float(&temperature) * TEMPHUM_SCALE_FACTOR),
```

Converting an out-of-range float to an unsigned integer is **undefined** in C. In this build the compiler
emits a saturating unsigned conversion, confirmed in the shipped object:

```text
$ objdump -d .../modules/temperature.c.obj
 298:   vmul.f32        s15, s15, s13
 29c:   vcvt.u32.f32    s15, s15        <-- saturates negative inputs to 0
```

`vcvt.u32.f32` clamps a negative operand to 0, so −5 °C goes out as `0` and the host decodes it as
**0.00 °C**. That is worse than an obviously-bogus value: it passes any plausibility check and silently
contaminates logged data. `jerrycan_types.h:194` documents the sensor range as −40 °C to +125 °C and instructs
the receiver to divide by 100, with no signed or offset convention on either side, so this cannot be fixed in
the firmware alone.

### 14. `degrees_to_pwm_count()` caches its scale factor in a shared static

**Affected:** both modules.

```c
// lib/motor_motion/motor_math.c:293-308
static uint32_t degrees_to_pwm_count(const servo_motor_context_t *context, const float degree,
                                     const bool calculate_scale) {
    static float scale_factor = 0;          // function-local, but one instance for all servos
    if (calculate_scale) {
        scale_factor = (context->max_angle_pwm - context->min_angle_pwm) / FULL_RANGE_IN_DEGREES;
    }
    ...
}
```

`motor_motion_servo_generate_displacement_table()` recomputes it once at the head of each table (line 326) and
relies on it for the rest (line 350). Both boards have more than one servo, and the two callers of the
generator run in different threads: `servo_move_to_position()` on `main` (priority −2, from the CAN handler)
and `servo_work_calculation_handler()` on the motor workqueue (priority −3).

Both are cooperative, so this is not arbitrary preemption — but `main` reaches explicit scheduling points
inside the generation sequence. `ll_queue_servo_positions()` → `k_msgq_put()` invokes the scheduler, at which
point a ready higher-priority workqueue refill for servo B runs and installs B's scale factor; servo A's
second buffer is then computed with it. The effect is only visible once two servos are given different
`min/max_pwm_duration_us`, which is exactly what per-unit calibration produces.

### 15. Cross-port GPIO callback registration produces spurious `GPIO_READ` frames

**Affected:** pellet module.

`drivers/gpio/generic_gpios.c:546-560` builds one mask from the pin *numbers* of every readable pin across all
ports, then registers a single shared callback object on each pin's port:

```c
gpio_port_pins_t pin_mask = 0;
for (int i = 0; i < pin_count; i++) { pin_mask |= BIT(pins[i].pin); }
gpio_init_callback(&data->state_change_callback, ll_generic_gpio_state_change_isr, pin_mask);
for (int i = 0; i < pin_count; i++) { gpio_add_callback_dt(&pins[i], &data->state_change_callback); }
```

The pellet generic-gpios node spans GPIOC 4/5 and GPIOB 0/1/11–14
(`cerebellumlab_pellet_module.dts:61-81`), so the mask has bits 11–14 set. The door sensors are on GPIOC
13/14/15 (`:40-57`) with their own interrupts. Zephyr invokes every callback in a port's list whose mask
intersects the triggering pin, so door activity on GPIOC 13 or 14 also fires
`ll_generic_gpio_state_change_isr()` → `jerrycan_generic_gpio_tx()`, emitting an unsolicited `GPIO_READ` frame
from the ISR.

### 16. `get_can_node_id()` can install an indeterminate CAN address

**Affected:** both modules.

`lib/jerrycan/jerrycan.c:27-33`:

```c
uint32_t device_type;
ll_generic_gpio_read(gpio_dev, 0x3, &device_type);
uint32_t node_id;
ll_generic_gpio_read(gpio_dev, 0xC, &node_id);
```

Both locals are uninitialised and both return values are discarded. `ll_generic_gpio_read()` returns early
without writing `*value` when any `gpio_pin_get_dt()` fails (`generic_gpios.c:200-204`), so a read error leaves
the node id stack-derived and indeterminate. The module then installs its RX filters on that address and
transmits under it — it disappears from the bus rather than failing initialisation deterministically.

### 17. Driver failures are misreported to the host

**Affected:** pellet module.

- `jerrycan_rgb_led_msg_handler()` (`lib/jerrycan/modules/rgb_led.c:49-64`) updates the cached `red`/`green`/
  `blue` values *first*, then discards all three `led_set_brightness()` returns and returns 0 unconditionally.
  A PWM failure is acknowledged as success and then reported as though it took effect in every subsequent
  periodic `RGB_LED` frame.
- `set_channel()` (`lib/jerrycan/modules/analog.c:106`) does report failure, but returns `-ret` where `ret` is
  a `ll_analog_out_error_t` enum (1–3). The host receives `-EPERM`, `-ENOENT` or `-ESRCH` — valid-looking
  errno values with meanings unrelated to what actually went wrong, against a contract that specifies a system
  errno. The client knows it failed but can draw the wrong conclusion about why.

### 18. The Halley's-method tolerance hard-codes 48 steps per revolution

**Affected:** pellet module.

`lib/motor_motion/motor_math.c:210`:

```c
const float position_tolerance = min_step / 48.0f / 8.0f;  // (steps/pulse) / (steps/rev) / 8
```

The comment says the divisor is steps/rev, but it is the literal 48 — `STEPPER_DEFAULT_STEPS_PER_REVOLUTION`,
not `context->steps_per_revolution`, which the host overwrites via `StepperCfgWrite`. Any configuration other
than 48 uses the wrong convergence criterion: at 200 steps/rev the accepted position error is up to ≈4.17×
the intended one. It sets the stopping threshold only — iterations often converge tighter, and the 16-iteration
cap can dominate — so the consequence is looser and less predictable pulse timing in the acceleration and
deceleration regions rather than a fixed error on every pulse.

---

## Unresolved — Needs the Host Contract

Two items where the firmware is internally inconsistent but the correct behaviour cannot be determined from
in-scope sources. Both are worth a decision; neither is counted as a finding.

- **Audio bin origin (Issue 12).** The loop's iteration count is sized for a DC-skipped stream and the pointer
  is not. If the host expects bins 1–64, every magnitude is attributed to the wrong frequency; if it expects
  0–63, the code is accidentally correct except for the dropped Nyquist bin. The protocol header carries a raw
  magnitude array with no bin-origin definition.
- **Absolute fixed-location range.** `stepper_save_fixed_location()`
  (`lib/motor_motion/motor_motion_workq.c:729-741`) clamps the relative branch to `[0, 15]` revolutions and
  applies no bound at all on the absolute branch, which also accepts negatives that the relative branch
  excludes. The value is persisted and is what `FIXED_XYZ` drives to. Nothing in scope establishes which
  branch encodes the intended contract.

## Appendix — Robustness Notes (Not Findings)

Recorded because they sit on the CAN command path and each is one missing line, but they do **not** meet the
criteria above: each requires an invalid client argument or a caller that does not exist today.

- `servo_attach_handler()` / `servo_detach_handler()` (`lib/jerrycan/modules/servo.c:97-121`) read
  `msg->servo_move.motor_id` — the 2-bit bitfield from the *move* payload — where the wire type is
  `jerrycan_cmd_servo_attach_t { uint8_t motor_id; }`. Values 0–2 coincide on both boards. Neither handler
  NULL-checks `servo_motor_by_id()` before `ll_servo_enable()`, so `motor_id = 3` would fault.
- `stepper_home()` (`motor_motion_workq.c:846-851`) takes `&work_context->context` before testing, then tests
  the address of a struct member against NULL. `stepper_cancel_all_work()`, `servo_cancel_all_work()`
  (`:384-401`) and `stepper_motor_stop()` (`motor_motion.c:22-27`) use `find_*_context_from_device()` without
  the NULL check that function is written to require. `stepper_set_parameters()` (`:716`) omits the
  `stepper_driver_device` NULL check that the init path at `:543` performs.
- `generic_gpios.c` registers one `gpio_callback` object on multiple ports, which shares a single list node
  between two port lists. It is benign in the current init order (the shared node ends up as the tail of
  GPIOC's list and the only entry in GPIOB's), but it is one added callback away from splicing them.

## Revision Notes

Revised against an independent evaluation. Substantive changes:

- **Stated a Critical bar** (safety function, unsafe physical output, persistent outage) and re-sorted
  against it. Concurrent-ack loss, small-move rejection, tare and tone-ack are now Issues; `can_send` is
  promoted to Critical.
- **Corrected the board table.** The NAU7802 load cell is on the **magnet** module
  (`cerebellumlab_magnet_module.dts:200`), not the pellet module, as previously stated.
- **Tone frequency 0 promoted to Critical** after checking the target rather than assuming it. Revision 1
  claimed Zephyr leaves divide-by-zero trapping off and the reload simply wraps. It does not:
  `z_arm_fault_init()` sets `DIV_0_TRP` unconditionally on ARMv7-M, no fatal-error handler is overridden, and
  the module halts. The evaluation repeated the original mistake in the same direction.
- **Corrected the `can_send` mechanism.** The block is on the unconditional `k_sem_take(&ctx.done, K_FOREVER)`
  in Zephyr's synchronous shim, on the first send; mailbox exhaustion is not required.
- **Removed the `FIXED_XYZ` `MONITOR_*` race** — its example contradicted Issue 2, which puts a floor of
  `steps_per_revolution` (≥48) timing entries on any accepted move, far more than can drain inside one timer
  callback. Replaced with the deterministic `FIXED_XYZ`-during-a-move defect, now folded into Issue 1.
- **Narrowed** Issue 10 to `min_angle` (the `angle_adjustment` writer is unreachable on both boards, and
  `fixed_position` is a stepper setting), Issue 12's consequence, and Issue 18's consequence.
- **Strengthened** Issue 7 (garbage status on *every* bootloader data response, and the stale-uuid mechanism
  on the generic ack) and Issue 13 (verified by disassembly: negative temperatures saturate to 0.00 °C rather
  than producing a large positive value).
- **Moved** the NULL-deref cluster and servo attach/detach to a non-counted appendix, and the two
  contract-dependent items to "Unresolved", per the stated criteria.
- **Dropped** citations to `software/libjerrycan/src/libjerrycan.cpp`, which is outside the direct-inclusion
  scope; the protocol header plus the premise that every CAN command is live establishes reachability.

Two evaluation points were checked and **not** accepted:

1. That the load cell and temperature sensor are on different boards and so do not share a system workqueue.
   Both are on the magnet module (`cerebellumlab_magnet_module.dts:188-192`, `:200-211`); the original claim
   in Critical 3 stands.
2. That the tone divide-by-zero yields a wrapped reload value rather than a fault. See above — it traps.
