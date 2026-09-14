# Stepper Motor Safety Analysis

Scope: code paths that can make a stepper grind, stall, overheat, run away, or wedge, restricted to states
reachable from JerryCAN traffic on the pellet module (`CONFIG_DT_HAS_LL_STEPPER_ENABLED`, three
`ll,stepper` instances on TIM1/TIM8/TIM20).

Files examined: `drivers/motor/{stepper.c,motor_common.c,adi_tmc2209.c}`,
`lib/motor_motion/{motor_motion_workq.c,motor_math.c,motor_motion.c,motor_settings.c}`,
`lib/jerrycan/{jerrycan.c,modules/stepper.c}`, `boards/arm/cerebellumlab_pellet_module/*.dts`,
`software/libjerrycan/inc/jerrycan_types.h`.

## Hardware Numbers Used Throughout

From the board DTS, all three stepper timers run `st,prescaler = <849>`:

```text
timer tick       = (849 + 1) / 170 MHz  = 5 µs      (context->timer_increment)
ARR written      = table[i], low 16 bits only       (DMABURST to ARR, TIM1/8/20 are 16-bit)
ARR = 1          → 100 kHz step rate                (the generator's lowest accepted value)
ARR = 10         → 20  kHz step rate                (the generator's out-of-range fallback)
ARR = 0          → 200 kHz step rate                (nothing rejects this on the homing path)
ARR = 65535      → 3.05 Hz step rate
```

Stepper positions are in **revolutions**. One generated pulse advances
`1 / (microsteps × steps_per_revolution)` revolutions (`motor_math.c:402`).

---

## Critical — Can Drive the Motor Into a Hard Stop or Past Its Pull-Out Torque

### C1. There Are No Soft Travel Limits, and the Protocol Has No Field for Them

`stepper_move_to_position()` (`lib/motor_motion/motor_motion_workq.c:743`) validates that the target is
finite and that velocity/acceleration are positive. It never compares the target against any travel
envelope, because none exists: `jerrycan_stepper_cfg_t`
(`software/libjerrycan/inc/jerrycan_types.h:153-166`) carries `microsteps`, `steps_per_revolution`,
`motor_max_velocity`, `motor_max_acceleration`, `homing_velocity` and `flip_limit_orientation` — and no
min/max position. The servo config struct has `min_position`/`max_position`; the stepper one does not.
The file header comment in `lib/jerrycan/modules/stepper.c:13-14` still claims cfg writes "set stepper
parameters such as minimum and maximum positions"; nothing does.

Reachable as: `JERRYCAN_CMD_STEPPER_MOVE`, `abs_or_rel = RELATIVE`, `position = -1000.0`. The planner
accepts it and emits ~1000 × `steps_per_revolution` × `microsteps` pulses. With the default 48 steps/rev
and 1 microstep that is 48,000 pulses; at the default 20 rev/s the axis drives into its mechanical stop
and keeps pulsing for 50 s. The TMC2209 is in StealthChop/SpreadCycle current control with `IRUN = 10`,
so it holds current through the whole stall.

A second route to the same place: `stepper_save_fixed_location()`
(`motor_motion_workq.c:729-741`) clamps to `[0, 15]` **only on the relative branch**. An absolute save
writes the value straight through:

```c
if (is_absolute) {
    context->fixed_position = position;          // no clamp, no finiteness check
} else {
    float val = context->fixed_position + position;
    context->fixed_position = (val <= 0.0f) ? 0.0f : (val > 15.0f ? val = 15.0f : val);
}
```

`fixed_position` is the target used by `attempt_motor_move()` (`lib/jerrycan/modules/stepper.c:140`), so
`STEPPER_MOVE{save=1, abs, position=1e6}` followed by `FIXED_XYZ` commands a million-revolution move.
It is also persisted to NVS, so it survives a reboot.

**Fix:** add `min_position`/`max_position` to `jerrycan_stepper_cfg_t` and clamp (or reject) in
`stepper_move_to_position()`; apply the `[0, 15]` clamp on both branches of
`stepper_save_fixed_location()` and reject non-finite values there.

### C2. The Limit Switch Protects Only the Transition, Not the State

`ll_motor_init()` configures the limit pin `GPIO_INT_EDGE_TO_ACTIVE`
(`drivers/motor/motor_common.c:172`), and the only stop logic lives in the
`LL_MOTOR_EVENT_LIMIT_SWITCH` branch (`motor_motion_workq.c:308-329`). The switch state itself is read
in exactly one place on a motion path — the "already touching" early return in `stepper_home()`
(`motor_motion_workq.c:860`). `stepper_move_to_position()` never reads it.

So once the carriage is resting on the switch there is no further edge, and a move deeper into the stop
runs unimpeded. With C1 this is the most direct grinding path in the firmware:

```text
1. JERRYCAN_CMD_STEPPER_HOME motor 0        → carriage sits on the switch, position := 0
2. JERRYCAN_CMD_STEPPER_MOVE rel -5.0       → direction BACKWARD, no new edge, no state check
3. 5 × 48 × microsteps pulses into the hard stop
```

`ll_stepper_get_limit_switch_state()` also returns `0` ("clear") when the GPIO read fails
(`drivers/motor/stepper.c:121-124`), so a pin fault reads as "safe to move".

**Fix:** read the switch in `stepper_move_to_position()` and refuse a move whose direction is toward an
already-asserted limit; use `GPIO_INT_LEVEL_ACTIVE` or re-check the level inside the ISR callback.

### C3. Homing Has No Distance or Time Budget

`stepper_home()` (`motor_motion_workq.c:846-893`) queues one 100 ms slug of constant-rate pulses and
sets `motion_calculation_done = false`. Nothing ever sets it true on the homing path. The
`LL_MOTOR_EVENT_DMA_QUEUE_EMPTY` handler re-arms the generator whenever `move_control == MOVING_HOME`
(`motor_motion_workq.c:300-304`), so the pulse train regenerates indefinitely. The **only** terminating
condition is the limit-switch edge.

If the switch is disconnected, its pin fails, the carriage jams before reaching it, or the coupling
slips, the motor runs at `homing_velocity` forever with no watchdog, no step budget and no way to abort
over CAN (see H2). The motion is also, by design, a hard start at full homing speed with no
acceleration ramp — see the table in C4.

**Fix:** cap homing at a configured maximum travel (revolutions or elapsed time) and fail the command
with `-ETIMEDOUT` when exceeded.

### C4. The Homing Table Is Completely Unclamped

Compare the two generators. The positional one bounds every entry
(`lib/motor_motion/motor_math.c:413-424`); the homing one does not
(`motor_motion_workq.c:429-446`):

```c
size_t stepper_generate_table_for_homing(const struct stepper_work_context *context, uint32_t *buf) {
    const float seconds_per_pulse =
        1 / context->homing_velocity / context->motor_steps_per_revolution * context->context.min_step;
    size_t n_pulses = (size_t)floorf(100.0f / 1000.0f / seconds_per_pulse);
    if (n_pulses > STEPPER_BUFFER_SIZE) { n_pulses = STEPPER_BUFFER_SIZE; }
    for (size_t i = 0; i < n_pulses; i++) {
        buf[i] = lroundf(seconds_per_pulse / context->timer_increment);   // no floor, no ceiling
    }
    return n_pulses;
}
```

`homing_velocity` comes straight from `JERRYCAN_CMD_CFG_WRITE` and is accepted by
`stepper_set_parameters()` on the sole test `homing_velocity > 0.0f` (`motor_motion_workq.c:709-711`).
`+Inf > 0.0f` is true, so infinity is storable.

| `homing_velocity` (rev/s) | `seconds_per_pulse` | `buf[i]` | Resulting step rate |
| --- | --- | --- | --- |
| 20 (default) | 1.04 ms | 208 | 960 Hz — intended |
| 5000 | 4.2 µs | 1 | 100 kHz — instant stall |
| 50000 | 0.42 µs | **0** | 200 kHz (ARR = 0 → update every tick) |
| `+Inf` | 0 | **0** | 200 kHz, and `n_pulses` is `(size_t)floorf(inf)` — undefined |
| 0.001 | 20.8 s | 4.17e6 → **truncated to 16 bits** | aliased to ~20 kHz |

Two distinct hazards: `buf[i] == 0` contradicts the invariant the positional generator explicitly
protects ("Sending even a single zero over the DMA will stop the timer", `motor_math.c:418`), and any
value above 65535 is silently truncated by the 16-bit ARR, turning a deliberately slow homing pass into
a fast one. `n_pulses == 0` is also possible, and unlike the positional path
(`motor_motion_workq.c:472`, guarded with `if (ret > 0)`) the homing path queues a zero-length block
unconditionally (`motor_motion_workq.c:488`, `:888`).

**Fix:** apply the same `[1, UINT16_MAX]` clamp used in `motor_motion_stepper_generate_timing_table()`,
reject `n_pulses == 0`, and bound `homing_velocity` to a finite maximum in `stepper_set_parameters()`.

### C5. An Infeasible Velocity Becomes the Maximum Pulse Rate Instead of an Error

`motor_motion_stepper_generate_timing_table()` (`motor_math.c:411-424`):

```c
const float time_delta = (this_time - last_time) / time_step;

if (time_delta >= (float)UINT16_MAX) {
    table[i] = UINT16_MAX;
} else if (time_delta <= 1.0f || isnan(time_delta) || isinf(time_delta)) {
    LOG_ERR("Time_delta is out of range (%f) at position %f", ...);
    table[i] = 10;                       // 20 kHz, from whatever velocity we were at
} else {
    table[i] = (uint32_t)lroundf(time_delta);
}
```

When the requested pulse rate exceeds what the 200 kHz timer can express, the generator does not fail
the move — it substitutes the fastest interval it is willing to emit and continues. The substitution is
applied per entry, so the ramp the profile computed is replaced by a flat 20 kHz for every affected
entry, *including entries at the start of the move where the motor is at rest*. Going from standstill
to 20 kHz in one step is far beyond any stepper's pull-in rate: the rotor does not follow, the axis
sits there buzzing at full `IRUN` current, and the firmware's position belief advances anyway.

This does **not** require abusive input. With `microsteps = 256` (a legal value the TMC2209 accepts) and
the default `motor_max_velocity = 20.0f` (`motor_motion_workq.c:52`) and `steps_per_revolution = 48`:

```text
required rate = 20 × 48 × 256 = 245,760 pulses/s
required ARR  = (1 / 245760) / 5 µs = 0.81  →  ≤ 1.0  →  fallback 10  →  20 kHz
```

Every entry of the cruise phase takes the fallback. Nothing in the system relates `motor_max_velocity`
to the achievable rate for the current `microsteps`/`steps_per_revolution`, so the "max velocity" clamp
in `stepper_move_to_position()` (`motor_motion_workq.c:790-791`) provides no protection at all.

A NaN anywhere in the profile lands in the same branch. One way to get there: `steps_per_revolution` set
to a very large or infinite value (only `> 0.0f` is checked, `motor_motion_workq.c:719`) makes the
per-pulse advance underflow to zero, `this_time == last_time` for every entry, and the whole 1024-entry
table fills with `10`. A host that confuses "steps per revolution" with "microsteps per revolution"
(48 vs 12288) gets a 20 kHz burst rather than a rejected config.

Secondary effect: up to 1024 `LOG_ERR` calls per generated block, in deferred mode, which will overflow
the log buffer and drop unrelated diagnostics.

**Fix:** compute the minimum representable interval for the active `microsteps` /
`steps_per_revolution` / prescaler, reject the move with `-EINVAL` when `movement_max_v` exceeds it (or
clamp `movement_max_v` down to it before planning), and abort the block rather than substituting a
value when `time_delta` is out of range.

### C6. A Rejected Microstep Write Leaves Firmware and Driver Disagreeing

`stepper_set_parameters()` (`motor_motion_workq.c:713-717`):

```c
if (microsteps > 0) {
    context->microsteps = microsteps;                                   // stored unconditionally
    const ll_motor_cfg_t *stepper_config = dev->config;
    adi_tmc2209_set_microstep(stepper_config->stepper_driver_device, microsteps);   // return ignored
}
```

`adi_tmc2209_set_microstep()` only accepts powers of two from 1 to 256 and returns `-EINVAL` otherwise
(`drivers/motor/adi_tmc2209.c:450-453`). It can also return `-EIO` if the CHOPCONF read-modify-write
over the single-wire UART fails. In either case the TMC2209 keeps its previous `mres` while
`context->microsteps` holds the new value, and `context->microsteps` is what every subsequent
`min_step = 1.0f / microsteps` calculation uses (`motor_math.c:123`).

`JERRYCAN_CMD_CFG_WRITE` with `microsteps = 100` (a plausible typo for 1/16 or 1/32) leaves the driver
at, say, 1 microstep while the planner emits pulses timed for 100. Each pulse moves 100× further than
the firmware believes, at 100× the intended velocity — the motor stalls immediately and the position
belief is wrong by two orders of magnitude, so the next absolute move has no relationship to reality.
The cfg write is acknowledged as successful because the return value is discarded.

There is no `motion_mode` guard on this function either, so the same write mid-move reprograms CHOPCONF
while the DMA is pulsing — a step-size change with no corresponding change in the queued timing table.

**Fix:** propagate the driver's return code, only commit `context->microsteps` when the write succeeded,
validate the power-of-two constraint at the CAN boundary, and reject cfg writes while
`motion_mode == MOTION_IN_PROGESS`.

---

## High — Wedges the Motor or Desynchronises Position (Grinding on the Next Command)

### H1. A Limit-Switch Stop Leaves a Stale Block in the Driver Queue and Wedges the Axis Permanently

A move longer than 1024 microsteps queues two blocks before returning
(`motor_motion_workq.c:808-830`). `ll_motor_start_dma()` pops the first to configure the transfer
(`drivers/motor/motor_common.c:62-71`), leaving the second in `stepper_msgq<n>`.

If the limit switch fires while that second block is still queued, `stepper_motor_stop()`
(`lib/motor_motion/motor_motion.c:22-27`) sets `MOTION_DONE` and calls `ll_stepper_disable()`, which
stops the DMA and disables the compare channel (`drivers/motor/stepper.c:86-93`). **Neither purges the
message queue.**

The next move then runs into this, in `ll_motor_queue_data()` (`motor_common.c:132-142`):

```c
int ret = k_msgq_put(cfg->msgq, &msg, timeout);
struct dma_status status;
dma_get_status(cfg->dma_dev, data->dma_channel, &status);
if (k_msgq_num_used_get(cfg->msgq) == 1 && status.busy == false) {
    ret = ll_motor_start_dma(dev);
}
```

With the stale block still present, `num_used` is 2 after the put, so the DMA is never restarted. The
queue can only drain by being consumed, and it can only be consumed by a DMA that will not start. The
axis is stuck:

- `stepper_move_to_position()` has already set `motion_mode = MOTION_IN_PROGESS`, so every later move
  returns `-EBUSY` (`motor_motion_workq.c:758-761`).
- `stepper_home()` returns `-EBUSY` at the same check (`:866`), so homing cannot clear it either.
- The motor never moves again until a power cycle.

There is a second failure mode on the same defect: the stale queue entry points into `buffers[0..1]`,
which the new move overwrites. If the DMA is ever restarted by some other path, it plays a timing table
belonging to a different profile — an arbitrary pulse train at arbitrary rates in whatever direction
the DIR pin currently holds.

**Fix:** purge the msgq (`k_msgq_purge`) in `ll_stepper_disable()`/`stepper_motor_stop()`, and have
`ll_motor_queue_data()` start the DMA whenever the channel is idle and the queue is non-empty rather
than only when it holds exactly one entry.

### H2. There Is No Way to Stop a Stepper Over CAN

`JERRYCAN_CMD_ESTOP` is defined in the protocol and sized in `jerrycan_msg_get_payload_size()`
(`lib/jerrycan/jerrycan.c:44`), but no module registers an RX callback for it. `trigger_e_stop()`
(`lib/motor_motion/motor_motion.c:45-49`) has **no callers anywhere in the tree**. The only stop
available is the shell command `cmd_stepper_stop` (`motor_motion_shell.c:320-342`), which requires a
console.

So every runaway described above (C1, C2, C3, C4, C5) has to be ridden out or cut at the power supply.
This also means the `e_stop_triggered` flag is never set, making the guard in
`stepper_move_to_position()` (`motor_motion_workq.c:763-767`) dead code — the "no motion after e-stop
without homing" property the comment describes is not enforced. Note that the guard is written as
`atomic_flag_test_and_set()` followed by `atomic_flag_clear()`, so even if the flag were used it would
transiently set it on every move.

**Fix:** register an ESTOP handler that calls `trigger_e_stop()`, and repair `motors_all_stop()` (M7)
before doing so.

### H3. `stepper_home()` Can Zero the Position Mid-Move

The order of checks in `stepper_home()` (`motor_motion_workq.c:860-869`) puts the "already touching the
switch" shortcut **before** the busy check:

```c
if (ll_stepper_get_limit_switch_state(dev) == 1) {
    LOG_INF("Already touching limit switch");
    stepper_set_position_to_zero(dev);        // rewrites end_pos and last_position_generated
    return 0;
}

if (work_context->motion_mode == MOTION_IN_PROGESS) {
    LOG_ERR("Attempted to move motor while already in motion.");
    return -EBUSY;
}
```

A `STEPPER_HOME` that arrives while the axis is moving away from an asserted switch takes the first
branch. `stepper_set_position_to_zero()` (`motor_motion_workq.c:232-241`) sets
`motion_profile.end_pos = 0`, `last_position_generated = 0` and `motion_mode = MOTION_DONE` — while the
DMA is still pulsing and the refill work is still scheduled. Consequences:

- The next refill computes `n_steps_to_finish = 0 - 0 = 0` and truncates the move.
- `MOTION_DONE` makes the 100 ms sweep ack the original move as complete
  (`lib/jerrycan/modules/stepper.c:159-172`) while the motor is still stepping.
- Because `motion_mode` is no longer `MOTION_IN_PROGESS`, a new move is accepted immediately and calls
  `ll_stepper_set_direction()` (`motor_motion_workq.c:780`) while the old pulse train drains — pulses
  from the old profile come out in the new direction.

The position belief is now wrong by however much of the old move was still in flight, which turns the
next absolute move into a crash into a hard stop.

**Fix:** move the busy check above the switch-state shortcut.

### H4. `moving_state` Is One Global for Three Motors and Two Command Types

`lib/jerrycan/modules/stepper.c:59` declares a single `moving_state`. It is written by
`stepper_move_handler()` (`:83`), `stepper_home_handler()` (`:321`) and `stepper_fixed_move()` (`:225`)
from the CAN RX thread, and by `stepper_handle_fixed_sequence()` / `stepper_handle_motion_complete()`
(`:174`, `:159`) from the 100 ms `k_timer` expiry — that is, from an ISR. There is no synchronisation.

Unconditional consequences, no race needed:

- **A single-motor move aborts an in-flight `FIXED_XYZ` sequence.** `stepper_move_handler()` sets
  `moving_state = MOVING_SINGLE` at `:83`, overwriting `MONITOR_X`/`MOVE_Z`/etc. The sequence stops
  advancing, the remaining axes never move, and the `FIXED_XYZ` command is never acknowledged. The host
  is left believing all three axes are positioned.
- **The state is set before the move is attempted.** `:83` runs before
  `stepper_move_to_position()` at `:89`. If that call returns `-EBUSY`, `-EAGAIN` or `-EINVAL`,
  `moving_state` is left at `MOVING_SINGLE` and the next motor to reach `MOTION_DONE` — any motor —
  gets acked with *its* `uuid` (`:166-169`).
- **`stepper_handle_motion_complete()` clears the state on the first completion it finds** (`:168`),
  so if two motors finish in the same 100 ms window only one ack is emitted.

There is also an ordering hazard: `stepper_handle_fixed_sequence()` checks `MONITOR_*` completion
first (`:184`, `:195`, `:206`) and only then sweeps every context, resetting `MOTION_DONE` → `MOTION_IDLE`
(`:163-165`). If a DMA completion lands between those two points, the sweep consumes the `MOTION_DONE`
the `MONITOR_*` state was waiting for and the sequence never advances.

**Fix:** move the sequence state into the per-motor context (or at minimum guard it), set it only after
the move is successfully started, and have the completion sweep consume `MOTION_DONE` only for the
motor the current state is actually waiting on.

### H5. The Motion Planner Runs Inside a Timer ISR

`K_TIMER_DEFINE(jerrycan_stepper_status_tx_timer, jerrycan_bulk_stepper_status_tx, NULL)`
(`lib/jerrycan/modules/stepper.c:372`) — Zephyr runs `k_timer` expiry functions in ISR context. That
handler calls `stepper_handle_fixed_sequence()` (`:361`) → `attempt_motor_move()` →
`stepper_move_to_position()`, so on a `FIXED_XYZ` tick the ISR performs:

- full profile initialisation with `sqrtf` and floating-point exception testing (`motor_math.c:45-89`),
- up to two 1024-entry timing tables, each entry running Halley's method with `cosf`/`sinf`
  (`motor_math.c:199-250`),
- `ll_queue_stepper_positions(..., K_FOREVER)` → `k_msgq_put()` with a blocking timeout from an ISR,
  which violates the kernel's contract (`motor_motion_workq.c:822`). `CONFIG_ASSERT` is off in this
  build, so it does not trap; the queue has 16 slots and only two are used, so it happens not to block
  today. It is one queue-depth change away from being a fatal fault in an ISR.
- `LOG_INF` with `%f` conversions (`motor_math.c:115-119`, `modules/stepper.c:164`) and three
  `jerrycan_tx()` calls.

The practical damage is the duration: this blocks same-priority interrupts, including the DMA
block-complete IRQs that drive buffer refill for the other two axes. A refill that arrives after the
current DMA block has drained produces the premature-completion behaviour in M2.

**Fix:** have the timer expiry submit to `motor_workq` and run the sequence logic from that thread.

---

## Medium

### M1. The "Already There" Threshold Is Wrong by a Factor of `steps_per_revolution`

`motor_motion_workq.c:769-772`:

```c
if (fabsf(target_position - context->context.last_position_generated) < 1.0f / context->microsteps) {
    LOG_WRN("Target position is the same as current position.");
    return -EAGAIN;
}
```

Positions are revolutions; one microstep is `1 / (microsteps × steps_per_revolution)` revolutions. The
threshold is `1 / microsteps` revolutions — too large by the full `steps_per_revolution` factor. At the
defaults (48 steps/rev, 1 microstep) every move under **one full revolution** is rejected. At 16
microsteps the dead zone is still 48 microsteps wide.

In the `FIXED_XYZ` path this is worse than a rejection: `attempt_motor_move()`
(`lib/jerrycan/modules/stepper.c:143-146`) treats the `-EAGAIN` as a hard failure, forces
`motion_mode = MOTION_DONE`, and the sequence silently skips that axis.

**Fix:** `1.0f / (context->microsteps * context->motor_steps_per_revolution)`.

### M2. `MOTION_DONE` Is Declared Without Checking Whether Planning Finished

`motor_motion_workq.c:293-298`:

```c
case MOVING_POSITION:
    if (context->motion_mode == MOTION_IN_PROGESS) {
        context->motion_mode = MOTION_DONE;
    }
    break;
```

`motion_calculation_done` is not consulted. If the DMA drains before the refill work has queued the next
block — see H5 for a mechanism that makes this reachable — the move is reported complete while the
generator still has work pending. That pending work then calls `ll_queue_stepper_positions()`, which
finds `num_used == 1 && !busy` and **restarts the DMA after the move was acked**. If the host has
already issued the next command, `ll_stepper_set_direction()` has flipped DIR, and the resumed pulses
from the old profile run backwards.

**Fix:** only transition to `MOTION_DONE` when `motion_calculation_done` is set; otherwise treat the
drain as an underrun and abort the move.

### M3. Short CAN Frames Are Not Length-Checked, and the Frame Buffer Is `static`

`lib/jerrycan/jerrycan.c:166-172`:

```c
static struct can_frame frame;          // line 109 — reused across every RX
...
msg.type = frame.id >> 5;
uint8_t msg_len = jerrycan_msg_get_payload_size(msg.type);
memcpy(msg.payload, frame.data, msg_len);
```

`frame.dlc` is never compared against `msg_len`. A 1-byte `STEPPER_MOVE` frame causes 13 bytes to be
copied out of a buffer holding one byte of new data and twelve bytes left over from a previous message.
`position`, `max_velocity` and `max_acceleration` are then whatever those stale bytes decode to. The
read stays inside the 64-byte `can_frame` array, so there is no overflow — but the move target is
effectively arbitrary, which feeds straight into C1.

**Fix:** reject frames where `can_dlc_to_bytes(frame.dlc) < msg_len + sizeof(msg.uuid)`.

### M4. `+Inf` Passes Every Config Validation

`stepper_set_parameters()` (`motor_motion_workq.c:701-723`) gates each field on `> 0.0f`. `NaN > 0.0f`
is false so NaN is correctly dropped, but `+Inf > 0.0f` is true. `motor_max_velocity`,
`motor_max_acceleration`, `homing_velocity` and `steps_per_revolution` can all be set to infinity over
CAN. C4 and C5 show where that lands. `motor_settings.c`'s `read_float()` (`:85`) has the same gap on
the NVS restore path.

**Fix:** use `isfinite(x) && x > 0.0f`, with per-field upper bounds.

### M5. `stepper_set_parameters()` Dereferences a Possibly-NULL Driver Device

`motor_motion_workq.c:715-716` passes `stepper_config->stepper_driver_device` to
`adi_tmc2209_set_microstep()` with no NULL check. The DT macro sets that field to `NULL` when
`driver-dev` is absent (`drivers/motor/stepper.c:151-152`), and the init path *does* check for it
(`motor_motion_workq.c:543`). All three pellet-module steppers declare `driver-dev`, so this is not
reachable on current hardware — but a stepper node without it turns any cfg write into a hard fault.

### M6. Limit-Switch DT Flags Need Verifying Against the Schematic

All three instances declare `limit-switch-gpios = <&gpioX N (GPIO_PULL_DOWN | GPIO_ACTIVE_LOW)>`
(`cerebellumlab_pellet_module.dts:273`, `:298`, `:323`), and the driver arms
`GPIO_INT_EDGE_TO_ACTIVE`. As written, the internal pull-down holds the line low and `GPIO_ACTIVE_LOW`
makes low the *active* level — so an open (disconnected or unpressed, depending on wiring) switch reads
as permanently asserted and the active edge never occurs.

If that is the actual behaviour: `stepper_home()` takes its "already touching limit switch" shortcut
(`motor_motion_workq.c:860`) and zeroes the position **wherever the carriage happens to be**, without
moving. Every subsequent absolute move is then offset by the true home distance, and drives into a hard
stop. This should be confirmed against the schematic rather than assumed either way — pull-down with
active-low is an unusual enough combination that it is worth an explicit answer.

### M7. `motors_all_stop()` Iterates the Wrong Array

`lib/motor_motion/motor_motion.c:35-43`:

```c
for (size_t i = 0; i < ARRAY_SIZE(servo_motors); i++) {
    servo_motor_stop(stepper_motors[i]);      // stepper_motors, not servo_motors
}
```

`set_all_e_stop_flags()` has the matching defect, and it is a **write**
(`motor_motion_workq.c:916-918`):

```c
for (size_t i = 0; i < ARRAY_SIZE(servo_contexts); i++) {
    atomic_flag_test_and_set(&stepper_contexts[i].e_stop_triggered);   // stepper_contexts
}
```

On the pellet module both arrays have three entries, so there is no out-of-bounds access — the e-stop
just stops the steppers twice and never stops the servos. On the magnet module (two servos, zero
steppers) `stepper_contexts[]` is zero-length and both loops run off the end, the second one writing
into whatever follows it in `.bss`. Both functions are currently unreachable because `trigger_e_stop()`
has no callers (H2) — but H2's fix makes them live.

### M8. Robustness Gaps in the `FIXED_XYZ` Path

`set_uuid_for_xyz_context()` (`lib/jerrycan/modules/stepper.c:126-133`), `attempt_motor_move()`
(`:135-149`) and `is_motor_motion_complete()` (`:151-157`) all dereference the result of
`find_stepper_context_from_device()` without a NULL check, and the first hardcodes motor IDs 0-2 rather
than using `STEPPER_COUNT`. This is safe today only because `modules/stepper.c` is built only when
`CONFIG_DT_HAS_LL_STEPPER_ENABLED` is set and the one such board has exactly three steppers. A board
with one or two steppers would take a NULL write on a single `FIXED_XYZ` frame — which is a broadcast
message with no `motor_id` field, so it reaches every node on the bus.

`is_motor_motion_complete()` also ignores its `state` parameter and reads the global `moving_state`
instead (`:152`). Harmless today since the caller always passes that same global, but it hides the
dependency.

---

## Suggested Order of Work

1. **C2 + C1** — read the limit switch before committing to a direction, and bound the target. Together
   these close the most direct grinding path.
2. **C4 + C5** — clamp the homing table and refuse infeasible velocities instead of substituting the
   fastest interval. These are where a plausible config typo becomes a stalled motor.
3. **H1** — purge the driver queue on stop; without this any limit-switch save bricks the axis until
   reboot.
4. **C3 + H2** — a homing travel budget and a working CAN e-stop, so a runaway is recoverable at all.
5. **C6, H3, H4, M1, M2** — the position-belief corruptions. Each one turns a later, perfectly
   reasonable absolute move into a crash into a hard stop.
6. The remaining medium items.
