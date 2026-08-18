# Flashing

How to get firmware onto a board that has never been programmed. Everything below refers to the
pellet module hardware rev v1.2 and magnet module hardware rev v1.1 unless stated otherwise.

## 1. Flashing a Bare Board

A board with blank flash has neither MCUboot nor an application, so the CAN updater
(`software/jerrycan_updater`) is not an option — it uploads into MCUboot's secondary slot and
needs MCUboot already resident. There are two ways to make the first load.

### Option A — SWD Through the Tag-Connect Pads (J1)

This is the path the repo is set up for.

**What you need**

| Item | Notes |
|---|---|
| ST-LINK-V3 probe | STLINK-V3MINIE or V3SET. Their STDC14 header is what J1 is wired to. An ST-LINK/V2 (20-pin 0.1") needs an adapter. |
| Tag-Connect **TC2070-IDC-NL-050** cable | The `-050` variant has the 0.05" debugger-end connector that mates with STDC14. See the note on "NL" below. |
| Physical access to J1 | The pad pattern on the board — no connector is fitted, and none can be. |
| Board powered normally | Barrel jack J22 on the pellet module, J20 on the magnet module, or 12 V on the CAN connector. J1/J9 pad 3 is the probe's target-voltage reference, not a supply. |
| `pyocd pack install stm32g4` | One-time, per the main README. |

**The cable does not clip on.** `-NL` stands for *No Legs*: that variant has no retaining clips, so
you hold it against the pads for the duration of the erase/program cycle. The legged
`TC2070-IDC-050` cannot be substituted — J1's footprint has only three Ø0.99 mm alignment holes
and no leg-clearance holes, so retrofitting one would take a board respin. Tag-Connect sells a
GRIP-14 retainer for hands-free use with the NL cable.

**J1 pinout** (ST's STDC14, drawn in the schematic as an `STLINK-V3-MINIE`):

| Pad | Net | Pad | Net |
|---|---|---|---|
| 1 | not connected | 8 | not connected (SWO) |
| 2 | not connected | 9 | not connected |
| 3 | 3V3 (reference) | 10 | not connected |
| 4 | SWDIO | 11 | not connected |
| 5 | GND | 12 | NRST |
| 6 | SWCLK | 13 | DEBUG_RX → LPUART1_RX (PC0) |
| 7 | GND | 14 | DEBUG_TX → LPUART1_TX (PC1) |

Pads 13/14 carry the console UART over the same cable; see [LOGGING.md](LOGGING.md).

**Flashing**

```bash
cd firmware/pellet_module
west build --sysbuild --board cerebellumlab_pellet_module -p
west flash
```

`runners.yaml` sets `flash-runner: pyocd`; `openocd` and `stm32cubeprogrammer` are also configured
if you prefer one of those:

```bash
west flash --runner stm32cubeprogrammer
```

Because the build is a sysbuild, this writes both images: MCUboot at `0x08000000` and the signed
application at `0x08010000`.

**If you only have the individual test points**, the same three signals are broken out separately:
TP48 = SWDIO, TP49 = SWCLK, and TP47 = MCU_RST. Also connect ground through J1 pad 5 or 7 and the
probe's target-voltage reference to 3V3 through J1 pad 3 or another 3V3 point. That is a
solder-and-leave option, which the NL cable is not.

### Option B — USB DFU Through the USB-C Port (J20)

The STM32G4's ROM bootloader can program a blank part over USB, with no debug probe involved.

**What you need**

| Item | Notes |
|---|---|
| A USB-C cable | J20 is a fitted Amphenol 12402012E212A receptacle, wired straight to the MCU's own USB (PA11/PA12). |
| `dfu-util` or STM32CubeProgrammer | USB mode. No probe, no Tag-Connect cable. |
| Access to J20, SW2, SW1 | All on the board. |
| **External power** | `USB_VBUS` connects to nothing but J20, so the cable cannot power the board. Barrel jack or CAN power must be up. |

**Entering the ROM bootloader**

1. Power the board.
2. Hold **SW2** — it connects BOOT0 to 3V3. R30 (10 k) holds BOOT0 at GND the rest of the time.
3. Tap **SW1** — RESET to GND.
4. Release SW2.

The bootloader then enumerates as a DFU device on PA11/PA12, which is exactly where J20's D+/D- go.
TP52 is the same BOOT net if you would rather fit a jumper than press a button.

**Writing both images** at their partition offsets:

```bash
dfu-util -a 0 -s 0x08000000 -D build/mcuboot/zephyr/zephyr.bin
dfu-util -a 0 -s 0x08010000:leave -D build/pellet_module/zephyr/zephyr.signed.bin
```

`0x08010000` is `slot0_partition`, which begins right after the 64 K `boot_partition`.

**Two caveats.** ST's AN2606, Table 117, documents USB DFU on PA11/PA12 for STM32G47x devices; confirm
the applicable bootloader version before depending on it in a production procedure. The ROM
bootloader also supports USART1/2/3, I2C2/3/4, and SPI1/2, but those pins are shared with application
hardware on these boards. This guide documents USB because the fitted USB-C receptacle makes it the
practical board-level route. The STM32G47x ROM bootloader does not provide an FDCAN interface.

### After the First Load

Once MCUboot and a signed application are resident, subsequent updates can go over CAN with
`software/jerrycan_updater`, and neither the probe nor the USB cable is needed.

## 2. Magnet Module

The same mechanisms apply, but use the magnet module's board name, build directory, and reference
designators:

| | Pellet | Magnet |
|---|---|---|
| Tag-Connect pads (SWD) | J1 | **J9** |
| USB-C receptacle | J20 | **J6** |
| Barrel jack | J22 | **J20** |
| RESET button | SW1 | **SW2** |
| BOOT0 button | SW2 | **SW3** |
| BOOT0 pull-down (10 k) | R30 | **R25** |
| SWDIO / SWCLK test points | TP48 / TP49 | **TP44 / TP43** |
| NRST test point | TP47 | **TP11** |
| BOOT0 test point | TP52 | **TP10** |
| USB D+ / D− test points | TP20 / TP19 | **TP36 / TP9** |

Confirmed identical in every respect this document depends on:

- J9 is the same `Tag-Connect_TC2070-IDC-NL_2x07` footprint with the same SWD wiring — 3V3 on pad 3,
  SWDIO 4, GND 5 and 7, SWCLK 6, and NRST 12. No connector is populated; hold the NL cable in place
  during programming or use the retainer.
- J6 is the same Amphenol 12402012E212A USB-C receptacle, fitted, with 5.1 k CC pull-downs (R2/R3)
  and ESD protection on D+/D− (U7) — and `USB_VBUS` again connects to nothing, so it cannot power
  the board either.
- Same flash map — `boot_partition` 64 K at 0, `slot0_partition` at `0x10000` — so the DFU offsets
  in section 1 are unchanged.

For SWD, build and flash from the magnet application directory:

```bash
cd firmware/magnet_module
west build --sysbuild --board cerebellumlab_magnet_module -p
west flash
```

For USB DFU, the MCUboot path is unchanged, but the signed application is
`build/magnet_module/zephyr/zephyr.signed.bin`.

## 3. NUCLEO-G474RE Differences

These differences are also covered in the main README's "Building the Pellet Module for a
NUCLEO-G474RE" section.

- **Flashing** is the on-board ST-LINK/V3E over the board's single USB connector (CN1). No
  Tag-Connect cable, no DFU dance, no external probe.
- **There is no MCUboot in that build**, so there is no CAN update path.
