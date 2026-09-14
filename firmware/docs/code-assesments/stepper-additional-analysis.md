# Additional stepper analysis: temporary recovery and oscillation

Assessed against commit `bae315c07a21723874c0b89e5e16117f1b0107b7` on 2026-09-14.
This narrows the [CX analysis](code-assesments/cx-stepper-analysis.md) and
[CC analysis](code-assesments/cc-stepper-analysis.md) using two additional observations:
replacement motors temporarily relieve grinding/stalling, and failures sometimes include
back-and-forth motion. Source, history, board component values, and manufacturer documentation
were checked; no failing hardware or CAN/STEP/DIR recordings were available. Suggested code
changes below are descriptions, not applied firmware changes.

## What the observations change

The strongest combined explanation is **marginal motion/drive settings, possibly aggravated by
incorrect stop/restart behavior**. A replacement can change friction, alignment, connection quality,
temperature, or rotor starting position enough to restore operation without removing the trigger.
If replacement includes powering down the controller, it also resets volatile firmware and driver
state. These possibilities must be separated before treating replacement as evidence of motor damage.

The same motors working with different firmware increases suspicion of drive settings and pulse
generation. It does not establish which firmware behavior is responsible unless supply, driver board,
mechanical load, and operating sequence are also comparable. There is no evidence here that firmware
has permanently damaged the motors.

**Back-and-forth motion does not establish that firmware is reversing DIR.** A rotor that loses
synchronism can rock while STEP continues in one commanded direction. Conversely, this firmware has
specific paths that allow DIR to change while an earlier move is still executing. These require
different investigations:

| Observation during failure | First suspects |
| --- | --- |
| DIR remains constant; rotor rocks or buzzes | Abrupt homing, torque/current margin, chopper behavior, intermittent phase connection |
| DIR changes while an earlier move is unfinished | Premature idle/completion and overlapping commands |
| Failure starts immediately after a limit stop or home | Incomplete stop, stale queue/refill work, old-rate pulses on re-enable |
| Failure starts after idle or warm-up | Freewheeling/position drift, current margin, driver state or connection changes |
| Failure coincides with a particular speed | Chopper transition, resonance, infeasible or discontinuous pulse timing |

There is no encoder feedback or autonomous position-correction loop in the examined stepper path.
Repeated deliberate reversals therefore need repeated commands, overlapping control paths, or another
external cause; they should not be described as a proven firmware “hunting” loop.

## 1. Abrupt homing remains the strongest established grinding trigger

**Confirmed behavior; strong symptom fit, especially if failures begin during homing.**

[`stepper_home()`](../lib/motor_motion/motor_motion_workq.c#L846) starts a constant-rate table without
an acceleration ramp. The default homing velocity is the default maximum movement velocity,
20 rev/s, not a separately conservative homing value
([defaults](../lib/motor_motion/motor_motion_workq.c#L188)). At 48 steps/revolution and full stepping,
the requested start is approximately 960 pulses/s, or 1,200 RPM from rest. Persisted configuration
can override these defaults; the actual deployed values need to be read.

This is supported by unusually direct historical evidence: commit `4b978ed8` says increasing maximum
velocity caused the motors to fail to move and grind during home, traced to the velocity jump from
rest. That change introduced a separate homing-speed setting but kept the constant-speed algorithm.

New or freshly remounted motors might tolerate that start where warmed, more heavily loaded, or
slightly misaligned motors do not. Loss of synchronism is compatible with rocking under constant
DIR. Once stalled, home keeps supplying blocks until a limit edge arrives, so an initial failure can
become sustained grinding. The missing timeout is an amplifier, not the initial explanation.

Homing also deliberately drains its only queued block before scheduling the next calculation
([generator/refill](../lib/motor_motion/motor_motion_workq.c#L429),
[queue-empty handler](../lib/motor_motion/motor_motion_workq.c#L300)). This introduces scheduling gaps
between nominally 100 ms blocks; it is not a continuously buffered constant-speed stream. The physical
importance depends on measured gap duration, but it adds another possible disturbance during home.

There is no universally correct one-line speed value. Lowering only a compiled default would not
change an already persisted homing speed.

## 2. Run current is substantially below the nominal-current comment; idle explicitly allows freewheeling

**Confirmed settings; strong torque/position-loss hypothesis, not proof of incorrect current selection.**

The driver selects `default_irun = 10`, `default_ihold = 1`
([initialization](../drivers/motor/adi_tmc2209.c#L588)).
[`adi_tmc2209_set_ihold_irun()`](../drivers/motor/adi_tmc2209.c#L395) subtracts one, so the hardware
fields are **IRUN=9 and IHOLD=0**. It also selects `vsense=1`, disables analog current scaling, and
sets `freewheel=1` ([GCONF](../drivers/motor/adi_tmc2209.c#L385),
[CHOPCONF](../drivers/motor/adi_tmc2209.c#L479), [PWMCONF](../drivers/motor/adi_tmc2209.c#L512)).
The [pellet v1.2 BOM](../../hardware/manufacturing/pellet-v1.2/BOM-pellet.csv#L45) lists 180 mΩ sense
resistors; the [stepper schematic](../../hardware/pellet_module/stepper.kicad_sch#L2542) agrees.

Using the manufacturer's UART current equation with those values gives
`I_RMS = (10/32) × 0.180/(0.180+0.020) / sqrt(2) ≈ 0.199 A`.
IHOLD=0 with FREEWHEEL=1 selects freewheeling after standstill reduction in StealthChop.
([TMC2209 datasheet, §§6.7, 9 and 20.2](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf))

Thus the nearby “353mA RMS ... nominal” comment is not the resulting configured current. The code
acknowledges the reduction, and history calls these settings empirically tested, so this is not
automatically a programming error. Nevertheless, roughly 0.20 A is a much more relevant starting point
for comparing the working firmware than assuming 0.35 A. This is a nominal calculation, conditional on
the installed board matching the repository and register writes actually succeeding.

Two plausible sequences deserve attention:

- Marginal run torque permits a freshly installed motor to work, then friction/load changes make the
  same commanded start fail. Increasing current without checking motor rating and temperature would
  not be an evidence-based fix.
- An axis backdrives while freewheeling, but firmware retains its old position. The next absolute move
  begins from the wrong physical location. This is particularly relevant to a gravity-loaded or
  mechanically backdrivable axis and failures after idle; it is not evidence of active reversal.

The source does not support blaming continuous full-current *idle* heating: it intentionally reduces
idle current/freewheels. Continued STEP during a stall is a different operating condition.

## 3. Chopper switching and tuning are meaningful firmware-specific suspects

**Confirmed configuration; physical causality remains conditional.**

The driver enables automatic StealthChop scaling/gradient tuning, supplies fixed `PWM_OFS=255` and
`PWM_GRAD=118`, and sets `TPWMTHRS=1500`
([PWMCONF](../drivers/motor/adi_tmc2209.c#L512), [threshold](../drivers/motor/adi_tmc2209.c#L605)).
These settings are shared across the three motors. Commit `5e7ee90` changed them after enclosure
testing; earlier history alternates between favoring StealthChop and SpreadCycle. The source itself
mentions troublesome motor resonances ([driver comment](../drivers/motor/adi_tmc2209.c#L628)).

TSTEP uses 1/256-step timing, independent of input microstep resolution. With the stated 12 MHz clock
and 48 steps/revolution, threshold 1500 corresponds to approximately 0.651 rev/s (39 RPM), before
hysteresis. Mode transitions can jerk; automatic tuning requires run-current standstill and suitable
motion. ([TMC2209 datasheet, §§5.2, 6.1 and 6.5](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf))

Consequently, the comment describing switching at high speed should not be read as “only near the
20 rev/s maximum.” Most sufficiently long accelerated moves cross this boundary. That is a useful
speed to correlate with recordings. Actual mode must be checked on the installed board, including its
SPREAD input and register readback.

Firmware does not explicitly verify completion of tuning before allowing movement. This is a missing
guarantee, not proof tuning fails: initialization time and previous motion may provide the necessary
conditions. Nor is automatic adaptation itself evidence of parameters progressively “going bad.”

Motor/load-dependent chopper behavior is credible: Analog Devices describes resonance-related
vibration and acceleration-dependent regulation limits in its
[StealthChop performance note](https://www.analog.com/en/resources/app-notes/an-015.html).
Comparing actual current and chopper configuration with the working firmware is more informative
than choosing either mode universally. A mode change is a diagnostic experiment, not an established
one-line repair.

## 4. A limit stop leaves the timer running, allowing old-rate pulses on restart

**Confirmed code path; especially strong if failure follows homing. Additional detail beyond the
previous stale-queue findings.**

[`stepper_motor_stop()`](../lib/motor_motion/motor_motion.c#L22) marks motion done and calls
[`ll_stepper_disable()`](../drivers/motor/stepper.c#L86). The latter stops DMA and disables the output
channel, but **does not stop the timer counter**. By contrast, ordinary queue exhaustion explicitly
disables the counter ([DMA callback](../drivers/motor/motor_common.c#L30)).

On the next accepted position move, firmware sets DIR, enables the output channel, and only then
generates and queues the new profile
([move startup](../lib/motor_motion/motor_motion_workq.c#L780)). If the counter remains running,
enabling the channel exposes pulses using its retained ARR, before the new timing data is ready.
The DMA-start function also deliberately avoids resetting a running counter
([startup](../drivers/motor/motor_common.c#L103)).

After a successful home this can mean an immediate start at the old homing rate, potentially in the
opposite direction, instead of the new move's intended acceleration. Pulse count depends on timing;
the source establishes the exposure window, not how many extra pulses occur on hardware. Unlike the
stale positional queue problem, this does not require a long move or a second queued block.

**Small local correction for this specific defect:** add
`LL_TIM_DisableCounter(cfg->timer);` in `ll_stepper_disable()` after disabling the output channel.
That prevents this retained-counter restart path. It does **not** complete stop/cancel handling:
pending refill work and queued buffers still need coordinated cancellation. In particular, stopping
home leaves `motion_calculation_done=false`, and an already scheduled homing refill does not check
whether motion is still active ([refill](../lib/motor_motion/motor_motion_workq.c#L483)).

Replacing the motor cannot clear these software states by itself. A power cycle performed during
replacement can. This distinction makes a power-cycle-only comparison particularly valuable.

## 5. Premature completion permits real mid-motion direction changes

**Confirmed control defects; best explanation if captured DIR actually reverses unexpectedly.**

Three paths from the earlier analyses survive source rechecking:

1. **`FIXED_XYZ` overlaps an active move.**
   [`attempt_motor_move()`](../lib/jerrycan/modules/stepper.c#L135) turns every negative return,
   including `-EBUSY`, into `MOTION_DONE`. The completion sweep then makes the axis idle without stopping
   its existing DMA. A later command is accepted and can change DIR and overwrite active buffers.
2. **HOME arrives while departing an asserted switch.**
   [`stepper_home()`](../lib/motor_motion/motor_motion_workq.c#L860) checks the switch before checking
   busy. Its already-home shortcut zeroes the position and marks motion done while pulses continue.
   Another move can then change DIR mid-flight. **Small local correction:** move the existing busy
   check above the already-touching-switch shortcut.
3. **The pulse queue drains before planning finishes.**
   [Queue-empty handling](../lib/motor_motion/motor_motion_workq.c#L288) marks a positional move done
   without checking `motion_calculation_done`. Pending refill work can subsequently queue more data.
   A new command can interfere with that work after the false completion. Whether this occurs in
   ordinary operation requires a timing trace; computational work in the
   [fixed-sequence status timer](../lib/jerrycan/modules/stepper.c#L360) makes it worth investigating.

These paths permit reversals; they do not autonomously generate endless alternating commands.
Repeated oscillation through them requires the relevant command/refill sequence to recur. Their fit
to replacement is primarily the accompanying controller reset, not the motor's identity.

Do not treat a queue purge or an extra completion-condition check alone as a complete repair of
ownership, cancellation, and concurrency across these paths.

## 6. Configuration can silently change the effective speed and torque demand

**Confirmed defects; higher priority if failure follows configuration or power-up.**

[`stepper_set_parameters()`](../lib/motor_motion/motor_motion_workq.c#L693) accepts configuration
during motion and stores microsteps before calling the driver, discarding its result. Invalid
microsteps or a failed driver update leave planned pulse counts inconsistent with physical stepping.
Even valid updates during movement change hardware behavior underneath already prepared timing data.
These can cause stalling with fixed DIR and explain system-to-system differences.

Two small corrections are independently clear, although neither completes configuration handling:

- Reject nonzero microstep counts outside the supported powers of two before changing any fields:
  `if (microsteps && (microsteps > 256 || (microsteps & (microsteps - 1)))) return -EINVAL;`
- In [`adi_tmc2209_write()`](../drivers/motor/adi_tmc2209.c#L143), replace its ignored helper call and
  unconditional success return with
  `return write_single_line_uart_and_flush_read(dev, datagram.raw, sizeof(datagram.raw));`.
  This exposes transport/echo errors; it does not prove the chip accepted the register write, and
  callers that discard errors still need attention.

The [timing generator](../lib/motor_motion/motor_math.c#L411) also replaces invalid/too-short intervals
with ARR=10. With 5 µs ticks, this is approximately **18.2 kHz**, using `(ARR+1) × tick`, rather than
the approximately 20 kHz stated in the older analyses. Legal configuration can reach this path:
20 rev/s × 48 × 256 requests 245,760 pulses/s. It is a discontinuous fallback, not a valid execution
of that requested profile. High pulse rate alone does not establish mechanical speed without knowing
the actual microstep resolution.

A persisted bad configuration survives a controller reboot and a motor replacement. It is therefore
a weaker explanation for temporary recovery unless the replacement also changes settings or merely
improves tolerance to the same excessive demand.

## 7. A phase connection fault remains a competing explanation that firmware does not detect

**Hardware hypothesis; confirmed absence of firmware supervision.**

Reseating connectors during replacement could temporarily restore an intermittent phase connection.
Rocking with steady DIR is compatible with this and should not be assigned to software without checking
coil currents. Manufacturer guidance warns that a disconnected phase under automatic StealthChop
scaling can overdrive the connected phase. Open-load flags alone are not conclusive.
([TMC2209 datasheet, §6.6.1](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf))

Firmware enables autoscaling but has no operational driver-check implementation:
[`stepper_work_check_driver_handler()`](../lib/motor_motion/motor_motion_workq.c#L496) is a TODO and
is not initially scheduled. It does not distinguish a stalled motor, missing phase, or thermal event
from successful pulse output. Status reports planned position and a constant zero status byte
([telemetry](../lib/jerrycan/modules/stepper.c#L336)).

This is an important blind spot, not evidence that a phase fault or overheating actually occurred.

## Most useful next evidence

1. **Capture STEP and DIR at the driver together with CAN traffic through one failure.** Constant DIR
   prioritizes issues 1–3 and 7; unexpected DIR transitions prioritize issue 5. Include the preceding
   home/limit event to expose issue 4. Record pulse gaps and intervals, not just average frequency.
2. **Separate replacement from its side effects.** Compare controller power-cycle only, cooling only,
   and reseating/remounting the original motor with replacement. Record whether recovery lasts minutes,
   hours, or many motion cycles. Different recovery patterns discriminate software state, temperature,
   connection, and mechanical explanations.
3. **Read actual settings on both working and failing systems.** Include persisted speed/acceleration,
   homing speed, steps/revolution, microsteps, and driver-register readback for current, chopper mode,
   and status. Compare coil current under the same supply and mechanical load.
4. **Correlate the first failure with startup, idle, speed, or limit events.** Capture temperature and
   driver fault state before resetting; the current CAN status cannot establish successful movement.

The earlier missing e-stop, unbounded homing, and travel-limit findings remain important containment
problems. They rank below the mechanisms above for explaining why a replacement helps temporarily or
why the rotor oscillates. Neither the older analyses nor this source review establishes permanent
motor damage or a single root cause without the distinguishing measurements.
