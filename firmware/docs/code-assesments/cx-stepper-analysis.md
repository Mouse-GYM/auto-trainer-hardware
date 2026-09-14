# Stepper Motor Safety Analysis

Assessment target: repository commit `5554033`.

## Scope and criteria

This review traces the three active pellet-module steppers from the public JerryCAN wire types through the CAN
handlers, motion work queue, trajectory generator, STM32 timer/DMA driver, TMC2209 configuration, limit-switch
handling, and settings persistence. The magnet module has no steppers. Findings require a sequence that can be
started with a defined CAN command; shell-only problems and purely theoretical future callers are excluded.

The focus is physical or operational risk: continued motion into a stop, unexpected direction or distance,
loss of the only remote stop mechanism, indefinitely running or indefinitely pending motion, and state
corruption that prevents safe recovery. Ordinary acknowledgement defects are included only where they can
cause unsafe retries or corrupt the motion state.

## Executive summary

The current stepper implementation is not fail-safe against a runaway. Homing deliberately generates pulses
forever until an interrupt arrives, but it has no travel/time budget; the defined CAN e-stop command has no
handler; and both configuration changes and `FIXED_XYZ` can make a moving motor ignore its home-limit edge.
There are also two independent ways for CAN configuration to desynchronize the planner from the TMC2209,
allowing moves to travel by a large multiple of the requested distance.

| Severity | Finding | Direct CAN trigger | Principal result |
|---|---|---|---|
| Critical | CAN e-stop is ignored | `ESTOP` during any move/home | No remote way to stop grinding or runaway motion |
| Critical | Homing has no deadline and its switch path fails open | `STEPPER_HOME` | Endless motion if the switch is missed, misoriented, or unavailable |
| High | Config writes are allowed during motion | `CFG_WRITE(STEPPER)` during `STEPPER_MOVE` | Limit bypass, instant step-scale change, unexpected travel |
| High | `FIXED_XYZ` can declare a physically moving axis idle | `FIXED_XYZ` during a move | Later limit edge ignored; active DMA buffers can be overwritten |
| High | Rejected TMC2209 microstep settings are accepted and persisted | `CFG_WRITE(STEPPER)` then move/home | Planner/driver scale mismatch and potentially massive overtravel |
| High | Position commands have no travel envelope | Large finite move or saved absolute target | Hours of commanded motion into a far-end mechanical stop |
| High | A limit stop leaves stale DMA work queued | Move reaches the home switch | False success, corrupt position, later move/home wedged until reboot |
| Medium | Numeric config can create zero-length or unsafe pulse trains | Extreme but valid positive configuration | Accepted command never completes, or motor is started far too fast |
| Medium | Driver faults and real position are not monitored | Any sustained move/home | Thermal/stall warning is invisible; telemetry can falsely imply completion |

## Detailed findings

### 1. Critical: the defined CAN emergency-stop command is silently ignored

`JERRYCAN_CMD_ESTOP` is a defined wire command and has a payload-size entry
(`software/libjerrycan/inc/jerrycan_types.h:28-70`, `firmware/lib/jerrycan/jerrycan.c:41-55`). JerryCAN dispatch,
however, invokes only registered callbacks (`firmware/lib/jerrycan/jerrycan.c:164-183`), and there is no e-stop
callback anywhere under `firmware/`. `trigger_e_stop()` exists but has no caller
(`firmware/lib/motor_motion/motor_motion.c:35-49`).

Reachable sequence:

1. Send a long `STEPPER_MOVE` or a `STEPPER_HOME` whose limit switch is not reached.
2. Send the library-supported `ESTOP` command.
3. The frame is decoded and discarded; timer/DMA output continues unchanged.

This turns every other runaway or grinding case in this report into a power-cycle/manual-intervention event.
The unused stop helper also only stops DMA/channel output; it does not purge queued motion work, so connecting
the handler without repairing queue cancellation would not make recovery safe. In addition, its servo loops
index the stepper arrays by mistake (`motor_motion.c:35-42`,
`firmware/lib/motor_motion/motor_motion_workq.c:911-918`), so the helper should not simply be wired to CAN as-is.

Recommended correction: register an e-stop handler that synchronously disables every step output, cancels
refill work, purges each motor queue, records that physical position is unknown, and rejects all position moves
until a successful bounded home. Repair the array-indexing defects before using the existing helper.

### 2. Critical: homing runs forever unless a limit interrupt arrives

`stepper_home()` sets `MOVING_HOME`, enables the output, and queues a constant-speed block
(`firmware/lib/motor_motion/motor_motion_workq.c:846-892`). Every DMA queue-empty event schedules another block
because `motion_calculation_done` remains false (`:288-305`, `:483-492`). There is no pulse count, maximum
travel, elapsed-time deadline, stall test, cancellation command, or watchdog. Only the limit-switch callback
ends the operation (`:308-327`).

This is reachable with one ordinary `STEPPER_HOME`. Any of the following then makes it genuinely endless:

- `flip_limit_orientation` was configured to the wrong value over CAN, so homing travels away from the only
  limit switch (`:873-880`);
- the switch or its wiring stays inactive;
- GPIO input/callback/interrupt setup failed. Initialization logs these failures but deliberately continues
  (`firmware/drivers/motor/motor_common.c:157-177`);
- a runtime GPIO read fails. `ll_stepper_get_limit_switch_state()` converts the error to "not active," allowing
  homing to begin (`firmware/drivers/motor/stepper.c:116-126`).

Once the carriage reaches a mechanical stop without producing the expected edge, firmware continues issuing
pulses indefinitely. Depending on motor torque and mechanics this is sustained grinding/stall heating. Finding
1 means the host cannot stop it with the protocol's e-stop command.

Homing also jumps directly from standstill to `homing_velocity`; it does not use the acceleration profile.
The value is CAN-configurable and accepted on the sole condition `> 0` (`motor_motion_workq.c:693-726`). There
is no finite check or hardware-specific maximum. The repository history itself records that raising homing
speed previously made these motors fail to move and grind (commit `4b978ed`). The new parameter moves that
failure behind configuration but does not validate it.

Recommended correction: impose a per-axis maximum homing distance and time, validate direction before motion,
fail closed on GPIO errors, use a safe bounded speed/acceleration profile, and end in an explicit fault state
that requires recovery rather than continuing to refill buffers.

### 3. High: a configuration write during motion can disable the limit stop or alter distance immediately

`stepper_cfg_write_handler()` accepts a configuration frame at any time and `stepper_set_parameters()` has no
busy-state guard (`firmware/lib/jerrycan/modules/stepper.c:235-260`,
`firmware/lib/motor_motion/motor_motion_workq.c:693-726`). Two fields are especially dangerous while DMA is
active.

First, the positional limit callback decides whether to stop using the *current* `flip_limit_orientation` and
the direction latched when the move started (`motor_motion_workq.c:319-325`). A deterministic sequence is:

1. With `flip_limit_orientation = false`, start a long backward move toward/past home. The latched direction is
   `LL_STEPPER_DIR_BACKWARD` (`:774-780`).
2. Before the limit is reached, write the same motor configuration with `flip_limit_orientation = true`.
3. When the home switch fires, both stop predicates at `:321-322` are false. The callback ignores the edge and
   DMA continues driving into the stop.

Second, a valid microstep change is written to the TMC2209 immediately while already-generated timing buffers
still encode the old scale. Changing, for example, from 256 microsteps to full steps partway through a move
makes every remaining pulse represent 256 times as much physical rotation as planned. The motion context's
`min_step` also remains the value captured at motion initialization, so asynchronous refills continue planning
with the old scale.

Recommended correction: reject all physical configuration changes while a motor is active, or stop and fully
invalidate/purge the move before applying them. The limit callback should use immutable per-move direction
metadata or, more safely, stop on the home edge whenever motion is toward home without consulting mutable
configuration.

### 4. High: `FIXED_XYZ` can make a running motor ignore its limit switch

The fixed-position sequencer uses one global `moving_state` for all axes. If its next axis is busy,
`attempt_motor_move()` treats every negative return, including `-EBUSY`, as though that axis were done and
writes `MOTION_DONE` (`firmware/lib/jerrycan/modules/stepper.c:135-149`). The same periodic callback then turns
that into `MOTION_IDLE` (`:159-170`) even though the original DMA transfer was never stopped.

A direct hazardous sequence is:

1. Start a long motor-0 move toward/past its home switch.
2. Send `FIXED_XYZ` while motor 0 is still active. This sets the sequencer to `MOVE_X` (`:223-228`).
3. On the next 100 ms status-timer pass, the X attempt returns `-EBUSY`; the sequencer marks motor 0 done and
   then idle while its original pulses continue (`:174-220`).
4. When motor 0 later reaches its limit, the callback's `motion_mode == MOTION_IN_PROGESS` guard is false, so
   the limit edge is ignored (`motor_motion_workq.c:308-328`). It keeps driving for the remainder of the
   original command.

There is a second consequence: because the context now says idle, another `STEPPER_MOVE` for motor 0 is
accepted while the first DMA transfer is physically active. It can change the DIR pin and regenerate the same
two backing arrays currently used by DMA/queued messages (`motor_motion_workq.c:743-835`). That creates
mid-motion reversal and live DMA-buffer corruption, with no predictable travel endpoint.

Recommended correction: never modify an axis's motion mode on `-EBUSY`; abort the whole fixed sequence with an
error or wait for ownership. Track sequence and completion state per motor, and serialize context/buffer
ownership against direct commands.

### 5. High: invalid microstep settings desynchronize software from the TMC2209 and survive reboot

For every nonzero CAN value, `stepper_set_parameters()` first writes `context->microsteps`, calls the driver,
discards the result, persists the software value, and returns success
(`firmware/lib/motor_motion/motor_motion_workq.c:713-726`). The TMC2209 accepts only
`{1, 2, 4, 8, 16, 32, 64, 128, 256}` and returns `-EINVAL` for every other value
(`firmware/drivers/motor/adi_tmc2209.c:410-457`).

After a rejected value, the planner uses `1 / configured_microsteps`, while the chip remains at its previous
resolution (`motor_motion_workq.c:790-794`, `firmware/lib/motor_motion/motor_math.c:121-125`, `:386-403`). For
example, starting from physical full-step mode and configuring `microsteps = 3` causes the next nominal
one-revolution move to emit three times the physical pulses required. Larger rejected values can produce much
larger overtravel. The invalid value is saved; at boot, the driver is initialized to full-step mode and the
invalid reapplication result is again ignored (`motor_motion_workq.c:536-545`), making the mismatch persistent.

Even a valid value is not reliably confirmed: `adi_tmc2209_write()` discards the UART transaction result and
always returns zero (`adi_tmc2209.c:143-163`). A failed write therefore creates the same software/hardware
mismatch while CAN reports success.

Recommended correction: validate the enum before changing context, propagate read/write errors, read back
`CHOPCONF.mres`, and persist/acknowledge only after the physical setting is confirmed.

### 6. High: move targets have no software travel envelope

`stepper_move_to_position()` rejects non-finite targets but places no minimum or maximum bound on any finite
absolute or relative target (`firmware/lib/motor_motion/motor_motion_workq.c:743-799`). The public CAN payload
uses an unrestricted `float` in revolutions (`software/libjerrycan/inc/jerrycan_types.h:98-115`). The only
installed switch per axis is the home-side limit; the positional callback intentionally ignores it while
moving away from home (`motor_motion_workq.c:319-325`). There is no far-end switch or software maximum in the
stepper context.

Saved targets have the same hole. The relative `save` branch clamps to `[0, 15]`, but the absolute branch saves
any float without validation; `FIXED_XYZ` later drives to it
(`motor_motion_workq.c:729-740`, `firmware/lib/jerrycan/modules/stepper.c:135-141`). This is an inconsistent
safety policy; the source does not document whether `[0, 15]` is the authoritative range for every axis, but
neither possible contract makes an unrestricted target safe for a finite-travel mechanism.

A finite target of `1,000,000` revolutions is fully encodable and, with the default 48 steps/revolution, is
only 48 million generated entries—within a 32-bit signed count. At the default 20 revolutions/second it
commands about 13.9 hours of motion. A mechanism with finite travel reaches its far stop early and receives
pulses for the remaining hours.

Recommended correction: define per-axis soft limits in physical units, require a valid home before accepting
absolute/fixed moves, range-check the computed absolute target of relative moves, and reject rather than clamp
or execute an out-of-envelope request.

### 7. High: a normal limit stop leaves stale queued buffers and cannot be recovered by homing

A positional move pre-generates up to two 1,024-entry blocks (`motor_motion_workq.c:801-830`). Starting DMA
removes only the first block from the message queue, normally leaving the second pending
(`firmware/drivers/motor/motor_common.c:62-83`, `:123-140`). On a limit edge, `stepper_motor_stop()` stops DMA
and disables the timer channel but neither it nor the callback purges that pending message or cancels refill
work (`firmware/lib/motor_motion/motor_motion.c:22-27`, `motor_motion_workq.c:308-325`,
`firmware/drivers/motor/stepper.c:86-92`).

The next move/home appends behind the stale entry. `ll_motor_queue_data()` restarts idle DMA only when the queue
contains exactly one message, so a queue that already contains stale work never restarts (`motor_common.c:132-140`).
The two queued pointers also refer to reusable context buffers, so a new calculation overwrites memory still
named by the stale queue entries. `stepper_home()` uses this same queue and therefore cannot recover the axis.

Meanwhile the planner advances `last_position_generated` when it creates a buffer, not when hardware executes
it (`firmware/lib/motor_motion/motor_math.c:386-432`). The limit callback does not correct the value, and the
periodic completion handler acknowledges `MOTION_DONE` as success (`firmware/lib/jerrycan/modules/stepper.c:159-170`).
The host is told the interrupted target succeeded while the motor's queue and position belief are both wrong.

Recommended correction: make stop/cancel atomically stop DMA, purge the message queue, cancel or drain refill
work, invalidate physical position, and complete the command with a distinct limit/fault error. Homing should
start from a guaranteed-empty queue.

### 8. Medium: accepted numeric configuration can create a permanently pending command or an unsafe pulse rate

The stepper config setter accepts every positive `float`, including `+Inf`, for maximum velocity,
acceleration, homing velocity, and steps/revolution. It checks neither finiteness nor plausible upper/lower
bounds (`motor_motion_workq.c:693-726`). Settings restoration also allows non-finite positive values through
its `value <= 0` test (`firmware/lib/motor_motion/motor_settings.c:72-86`, `:335-352`).

There are concrete CAN-reachable failures even without NaN or infinity:

- Configure valid full-step mode, 48 steps/revolution, and `homing_velocity = 0.1` rev/s. The homing generator
  computes `floor(0.1 seconds * 0.1 * 48) == 0` entries, then `stepper_home()` submits a zero-byte DMA block,
  ignores the queue/start return, leaves `MOTION_IN_PROGESS`, and reports that the command is pending
  (`motor_motion_workq.c:429-445`, `:873-892`). There is no reliable event that can complete it.
- Configure a very high homing speed. The first pulse block is emitted immediately at constant speed with no
  ramp. The timer count is rounded without a lower-bound check (`:441-443`); sufficiently high values round to
  zero. More moderately excessive values reproduce the documented stall/grinding failure discussed in
  Finding 2.
- Configure an extremely small positive `steps_per_revolution`, then issue an otherwise valid position move.
  The generated entry count truncates to zero (`firmware/lib/motor_motion/motor_math.c:386-395`), but the move
  was already marked in progress and the driver was enabled; with no DMA transfer there is no queue-empty
  completion callback (`motor_motion_workq.c:801-835`).

The positional generator has a related unsafe fallback: timing deltas at or below one tick, or any NaN/Inf
delta, are converted to a hard-coded 10-tick pulse interval instead of failing the command
(`motor_math.c:411-424`). With the active 200 kHz stepper timers, that is a 50 microsecond period. Bad motion
math therefore becomes a fast pulse train rather than a safe stop.

Recommended correction: validate every configuration as a finite, per-motor bounded value; require every
generated move/home block to contain at least one valid entry; propagate all queue/DMA returns; and treat an
unrepresentable timer interval as a command error rather than substituting a fast interval.

### 9. Medium: there is no runtime stall/thermal/fault supervision, and position telemetry is predictive

The TMC2209 exposes overtemperature prewarning/shutdown, short, open-load, and standstill fields in
`DRV_STATUS` (`firmware/drivers/motor/adi_tmc2209_types.h:42-63`). Production motion code never reads them.
`stepper_work_check_driver_handler()` is a TODO and merely reschedules itself, and initialization does not
schedule it in the first place (`firmware/lib/motor_motion/motor_motion_workq.c:496-503`, `:525-550`). Thus a
long move or endless home gets no firmware response to sustained stall heating; protection is left entirely
to the driver's hardware behavior.

The periodic status cannot serve as an external substitute. Its `status` byte is always zero, and its
`position` is `last_position_generated` (`firmware/lib/jerrycan/modules/stepper.c:336-351`). That position is
advanced during planning up to two buffers ahead of actual pulses (`motor_math.c:386-432`,
`motor_motion_workq.c:807-830`) and is not corrected after a limit stop. A short move can therefore report its
final target almost immediately while still moving, and an interrupted move can continue reporting the
unreached target.

Recommended correction: poll and latch driver faults during motion, stop on actionable fault/thermal states,
surface a real state/fault code over CAN, and track executed pulses (or explicitly report planned versus
executed position). Never report a limit-interrupted target as the physical position.

## Secondary control-path concern

One module-global `moving_state` tracks all three motors while UUIDs and physical state are per motor
(`firmware/lib/jerrycan/modules/stepper.c:47-59`). Concurrent move/home commands on different axes execute, but
the first observed completion clears the global state; later completed axes are set idle without their
completion acknowledgement (`:159-170`). A second command to a busy motor also overwrites the first command's
UUID before the busy check (`:69-101`). These are primarily protocol defects, but a host that retries missing
relative commands can produce repeated travel. Per-axis command ownership/completion records should replace
the shared state.

## Suggested remediation order

1. Implement a real CAN e-stop and a queue-safe atomic stop primitive.
2. Bound homing by time and travel, fail closed on limit/GPIO faults, and constrain its speed.
3. Serialize configuration, direct moves, homes, and `FIXED_XYZ` per axis.
4. Validate and confirm every TMC2209/motion configuration before committing it.
5. Add per-axis travel envelopes and executed-position tracking.
6. Add driver fault supervision and meaningful CAN fault/completion reporting.

## Verification notes

This is a source and active-configuration review. The pellet-module device tree confirms three enabled
steppers, each with one limit input and a 200 kHz timer
(`firmware/boards/arm/cerebellumlab_pellet_module/cerebellumlab_pellet_module.dts:253-325`). The public client
library exposes every command sequence used above (`software/libjerrycan/src/libjerrycan.cpp:236-268`,
`:315-375`, `:564-570`). No hardware-in-the-loop run was performed, so the exact acoustic/thermal outcome of a
stall depends on the mechanism and TMC2209 hardware protection; the continued-pulse, ignored-limit, scale
mismatch, and non-completing state transitions are established directly by the code.
