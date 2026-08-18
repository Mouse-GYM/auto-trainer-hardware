# Logging

How to get logging messages from the firmware. Everything below refers to the pellet module
hardware rev v1.2 and magnet module hardware rev v1.1 unless stated otherwise.

## 1. Viewing Logs on Real Boards

Two ports carry output, and they carry different things:

| Stream | Port |
|---|---|
| MCUboot output | LPUART1 → Tag-Connect |
| Zephyr boot banner, `printk` | LPUART1 → Tag-Connect |
| Application `LOG_*` messages | **USB-C CDC ACM** |
| Interactive shell | **USB-C CDC ACM** |

### The UART Side: LPUART1 Through the Tag-Connect

`zephyr,console = &lpuart1`, which is PC1 (TX) / PC0 (RX) at **115200 8N1**. Two ways to reach it:

- **ST-LINK VCP.** The probe pressed onto J1 for flashing also carries pads 13/14, so its own USB
  presents a virtual COM port on your PC alongside the debug interface. One cable, three functions:
  program, debug, console.
- **Test points.** TP51 = `DEBUG_TX` (PC1, the MCU's output → your adapter's RX) and TP50 =
  `DEBUG_RX` (PC0, the MCU's input ← your adapter's TX), with ground from J1 pad 5 or 7. Any 3.3 V
  USB-serial adapter works, and unlike the NL cable it stays put.

What you see there: MCUboot's output first, then the Zephyr boot banner, then any `printk`.
`pellet_module/sysbuild/mcuboot.conf` selects DEBUG-level MCUboot logging; the magnet module keeps
MCUboot at its default INFO level. Application `LOG_*` messages do **not** appear here — see the
consequences below.

### The USB Side: CDC ACM on the USB-C Port

`zephyr,shell-uart = &cdc_acm_uart0`. Plug a USB-C cable into J20 and the board enumerates a serial
port under FTDI's VID/PID (0x0403/0x6001) — `/dev/ttyACM0` or similar. This is the **default route
for application `LOG_*` messages**, because `CONFIG_SHELL_LOG_BACKEND=y` renders them into the
interactive shell session. Available commands depend on the application; the pellet module enables
the settings, `motor_motion`, and TMC2209 shells, while the magnet module has a different command
set.

`picocom -b 115200 /dev/serial/by-id/usb-FTDI_USB-DEV_0123456789ABCDEF-if00`

The pellet module's USB device settings live in
`pellet_module/boards/cerebellumlab_pellet_module.conf`; the magnet module's settings remain in
`magnet_module/prj.conf`. Both enable the USB device stack, CDC ACM, and initialization at boot.

**Consequences worth knowing**

- **Application `LOG_*` output does not reach the programmer's UART by default.**
  `CONFIG_LOG_BACKEND_UART` is not set — Zephyr
  leaves it off when the serial shell backend owns a console — so the shell backend is the only log
  backend in the image. A board with nothing plugged into the USB-C port prints MCUboot's output and
  the boot banner, then goes quiet. To get application log lines on *both* ports, add the following
  to `pellet_module/boards/cerebellumlab_pellet_module.conf` or `magnet_module/prj.conf`, as
  applicable:

  ```text
  CONFIG_LOG_BACKEND_UART=y
  ```

- **MCUboot's output never appears over USB.** It runs before the application, its console is
  `&lpuart1`, and it does not bring up the USB stack.
- **The earliest application messages are unreliable.** `CONFIG_LOG_MODE_DEFERRED=y` queues them,
  but enumeration takes time and `CONFIG_USB_CDC_ACM_RINGBUF_SIZE` is 1024 bytes — anything logged
  before your terminal opens the port can be dropped.
- **A crash can take the shell with it.** A fault in USB, in DMA, or anywhere before the log thread
  drains leaves you with nothing on that port. The UART remains available for MCUboot, the boot
  banner, and direct `printk`, but application logs do not fail over to it automatically.
- **CDC ACM is separate from SWD.** It requires a USB connection to J20/J6 rather than the probe's
  host connection. The same board-side USB-C connection can also be used for ROM DFU.
- **External power is still required**, since `USB_VBUS` is unconnected.

**To put the shell back on the UART**, add this to the application's devicetree overlay:

```dts
/ {
    chosen {
        zephyr,shell-uart = &lpuart1;
    };
};
```

---

## 2. Message Delivery and Levels

### The Default Path

Zephyr's `LOG_ERR` / `LOG_WRN` / `LOG_INF` / `LOG_DBG` calls hand off to a background log thread
(`CONFIG_LOG_MODE_DEFERRED=y`) which writes to the registered backends. Exactly one is active: the
**shell backend** (`CONFIG_SHELL_LOG_BACKEND=y`), which renders log lines into the interactive
session on `zephyr,shell-uart` — the USB-C CDC ACM port.

The UART backend (`CONFIG_LOG_BACKEND_UART`) is *not* enabled; Zephyr leaves it off when the serial
shell backend is in play. So `zephyr,console` carries the boot banner and `printk` but no application
`LOG_*` output. Setting `CONFIG_LOG_BACKEND_UART=y` in the applicable board configuration puts
application log lines on both ports.

`CONFIG_LOG_DEFAULT_LEVEL=3` (INFO), so `LOG_DBG` is compiled out unless a module opts into a
higher level. `CONFIG_LOG_MODE_OVERFLOW=y` with a 1024-byte buffer means a burst drops the oldest
entries rather than blocking.

### Raising an Individual Module's Level

Most custom drivers and libraries register with a level symbol. Set these in the applicable
application configuration (values are 0 = off, 1 = ERR, 2 = WRN, 3 = INFO, 4 = DBG):

| Symbol | Module name in the log | Default |
|---|---|---|
| `CONFIG_LIB_JERRYCAN_LOG_LEVEL` | `jerrycan`, `audio_in` | 3 |
| `CONFIG_LIB_MOTOR_MOTION_LOG_LEVEL` | `motor_motion` helper sources | 4 |
| `CONFIG_LL_MOTOR_LOG_LEVEL` | `ll_motor` (servo and stepper drivers) | 3 |
| `CONFIG_ADI_TMC2209_DEBUG_LEVEL` | `adi_tmc2209` | 1 |
| `CONFIG_LL_GENERIC_GPIO_LOG_LEVEL` | `ll_generic_gpios` | 4 |
| `CONFIG_LL_TONE_GENERATOR_LOG_LEVEL` | `ll_tone_generator` | 4 |
| `CONFIG_LL_ANALOG_OUT_LOG_LEVEL` | `ll_analog_out` | 4 |
| `CONFIG_LL_LOAD_CELL_LOG_LEVEL` | `ll_load_cell`, `nau7802_chip` | 4 |
| `CONFIG_LL_PRESSURE_SENSOR_LOG_LEVEL` | `ll_pressure_sensor` | 4 |
| `CONFIG_LIB_MIC_LOG_LEVEL` | `microphone` | 4 |

`motor_motion.c` and `motor_settings.c` register without an explicit level symbol, so those source
files use `CONFIG_LOG_DEFAULT_LEVEL` (3); `motor_settings` is also a separate module. The application
itself (`LOG_MODULE_REGISTER(app)` in `src/main.c`) and Zephyr's own subsystems likewise use
`CONFIG_LOG_DEFAULT_LEVEL`, or their subsystem symbol — `CONFIG_CAN_LOG_LEVEL`,
`CONFIG_DMA_LOG_LEVEL_DBG` and so on.

### Enabling the `log` Shell Command

`CONFIG_LOG_RUNTIME_FILTERING=y` is enabled (the shell backend pulls it in), but
`CONFIG_LOG_CMDS` is **not set**, so the usual `log enable dbg <module>` is unavailable and every
level change means a rebuild. Adding this to `prj.conf` gets it back:

```text
CONFIG_LOG_CMDS=y
```

### TMC2209 Default Level

`drivers/motor/adi_tmc2209.c` registers as
`LOG_MODULE_REGISTER(adi_tmc2209, CONFIG_ADI_TMC2209_DEBUG_LEVEL)`. That Kconfig symbol is defined in
`drivers/motor/Kconfig.adi_tmc2209` and deliberately defaults to **1 (ERR only)**, so its INFO and
DEBUG messages are compiled out. Set `CONFIG_ADI_TMC2209_DEBUG_LEVEL=3` or `4` in the pellet module
configuration to include them.

---

## 3. Magnet Module

The transport and backend layout is the same on the magnet module, although its enabled log modules
and shell commands differ. The relevant reference designators are:

| | Pellet | Magnet |
|---|---|---|
| Console through Tag-Connect | J1 | **J9** |
| USB-C receptacle | J20 | **J6** |
| Barrel jack | J22 | **J20** |
| Console TX / RX test points | TP51 / TP50 | **TP46 / TP45** |

The corresponding logging connections are:

- J9 carries DEBUG_RX on pad 13 and DEBUG_TX on pad 14, with ground on pads 5 and 7.
- J6 carries the default application-log and shell connection over USB CDC ACM. `USB_VBUS` is
  unconnected, so it cannot power the board.
- Same console split: `zephyr,console = &lpuart1` on PC1/PC0 at 115200 8N1, and
  `zephyr,shell-uart = &cdc_acm_uart0`.
- `&usb` is enabled with a `cdc_acm_uart0` child, and the same USB device settings under FTDI's
  VID/PID are present in `magnet_module/prj.conf`.

## 4. NUCLEO-G474RE Differences

Covered in the main README's "Building the Pellet Module for a NUCLEO-G474RE" section; the console
story is the part worth restating here.

- **All output is on the ST-LINK VCP.** Both `zephyr,console` and `zephyr,shell-uart` point to
  LPUART1 on PA2/PA3, so Zephyr boot output, `printk`, application `LOG_*` messages, and the
  shell all use the programmer's virtual COM port. Use `minicom -D /dev/ttyACM0` at 115200 8N1.
- **Do not move the console to `cdc_acm_uart0` there.** The STM32's own USB peripheral has no
  connector on a Nucleo-64; the `usb` node is disabled and its PA11/PA12 pins are carrying FDCAN1.
  `boards/nucleo_g474re.conf` turns the USB device stack off for this reason.
