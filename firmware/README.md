# Autotrainer Firmware

## Building the Firmware with Docker

The preferred build method uses the same Zephyr SDK container as CI. Clone the
repository on the host so the source, downloaded West dependencies, and build outputs
remain available after the container exits:

```bash
cd "$HOME"
git clone https://github.com/Mouse-GYM/auto-trainer-hardware.git
cd "$HOME/auto-trainer-hardware"
docker pull ghcr.io/mouse-gym/auto-trainer-hardware/zephyr-sdk:latest
docker run --rm -it \
  --volume "$(pwd):/auto-trainer-hardware" \
  --workdir /auto-trainer-hardware \
  ghcr.io/mouse-gym/auto-trainer-hardware/zephyr-sdk:latest
```

The remaining commands in this section run inside the container. Initialize the West
workspace and download the dependencies declared in `firmware/west.yml`:

```bash
cd /auto-trainer-hardware/firmware
west init -l
west update
```

Each native board produces two firmware images:

* MCUBoot
* The signed application

West sysbuild coordinates both builds.

### Building the Pellet Module for Its Native Board

```bash
cd /auto-trainer-hardware/firmware/pellet_module
west build -p --sysbuild -b cerebellumlab_pellet_module
```

The application images are:

* `firmware/pellet_module/build/pellet_module/zephyr/zephyr.signed.hex`
* `firmware/pellet_module/build/pellet_module/zephyr/zephyr.signed.bin`

The bootloader images are:

* `firmware/pellet_module/build/mcuboot/zephyr/zephyr.hex`
* `firmware/pellet_module/build/mcuboot/zephyr/zephyr.bin`

### Building the Magnet Module for Its Native Board

```bash
cd /auto-trainer-hardware/firmware/magnet_module
west build -p --sysbuild -b cerebellumlab_magnet_module
```

The application images are:

* `firmware/magnet_module/build/magnet_module/zephyr/zephyr.signed.hex`
* `firmware/magnet_module/build/magnet_module/zephyr/zephyr.signed.bin`

The bootloader images are:

* `firmware/magnet_module/build/mcuboot/zephyr/zephyr.hex`
* `firmware/magnet_module/build/mcuboot/zephyr/zephyr.bin`

After a module has been configured once, `west build` from that module's directory is
sufficient for subsequent builds. Use the full command above after deleting a `build`
directory or when changing boards.

## Flashing and Logging

See [FLASHING.md](FLASHING.md) for initial programming, USB DFU, and subsequent firmware updates.
See [LOGGING.md](LOGGING.md) for console connections, USB CDC ACM, and log-level configuration.

## Functional Description

### Commands and Data 
Both the pellet and magnet module are command driven with data streaming. They do 
not act on their own; there is no autonomous control of the devices under their purview.

Commands are sent over CANbus. When a command is _complete_ an acknowledgement is returned
on the CANbus. Most commands are resolved immediately, both others (e.g. motor movement)
wait until the action with the command is completed (e.g. the motor reaches its intended target
position).

Some commands request data (e.g. firmware version); the response to those commands 
return the requested data. In those cases, the command is not acknowledged separately.

Various data sets are streamed at different rates. The rates are specified in the project
file, but have defaults in the KConfig file for the lib/jerrycan module.

### Hardware Interfaces

#### Pellet Module
The pellet module controls and provides status for the following:
* X, Y, Z stepper motors
* Load arm and Cover/Barrier servo
* Door positions (open/close)
* RGB LED
* Analog output channels
* Stimulus control outputs
* Tone Generation

#### Magnet Module
The magnet module controls and provides status for the following:
* Manget head servo
* Microphone
* Temperature and Humidity sensor
* Load cell sensor
* Pressure sensor
* Head Fix detection

# Python Integration

## Libjerrycan

Libjerrycan is the hook between the Python application code and the CANbus that communicates
with the hardware. This code is part of the LeafLabs repository. It's essentially a
python binding for a C++ library that communicates on the bus. The source for this library lives
in the firmware autotrainer repo at `software/libjerrycan`

During development, it may be appropriate to make updates to the interface to the firmware
(new command or status features). The best way to incorporate changes in a local
development environment is to install it into the existing conda environment. This 
trick is the easiest way to install local changes to the libjerrycan library:

```shell
conda activate <name>
cd <parent>/autotrainer/software/
pip install ./libjerrycan
```

**Note**: To avoid inconsistencies in environments, it is recommended to activate 
the conda environment from a bare shell, not from the virtual environment that is needed
to develop the firmware.
  
# Bootloader

There is a bootloader for loading new images onto the pellet and magnet target boards.
The bootloader sends the image across the CANbus.

**Note:** the CANbus is assumed to be can0.

The bootloader lives in the autotrainer software repo at `software/jerrycan_updater`.

To build the application:
```bash
cd <parent>/autotrainer/software/jerrycan_udpater
cmake -B build -S .
cmake build
```

To run the application:
```shell
./build/jerrycan_updater -m <addr> -f <file>
# where
#   addr is the CAN address of the target. Add 4 for magnet modules.
#   file is the appropriate signed image file.
```

# Miscellaneous Notes

## CANBus

* For all applications, whether on the Jetson device or a local development machine, it's assumed that the CANbus is can0.
* If using a local development machine to communicate with installed magnet and pellet modules, disconnect the CAN connector on the JETSON and plug directly into that board.
* Beware the CANbus cables. Some are wired incorrectly. It may be all such cables have been cut in two, but be certain that the cable(s) you are using are the correct ones.

## Whisker-Wire Support Application

**The Whister-Wire Support Application is Deprecated**

Initial firmware development was accompanied by the development of a whisker-wire support
application. As integration with the CU team was initiated, the whisker-wire 
support application was not being maintained in preference to supporting and maintaining
the tools CU had started for integrating and testing the pellet and magnet modules.

## Using C-Lion as IDE

* Load, as a project, the CMakeLists.txt file at the Firmware directory. This gives 
  the full view of the source for the applications, zephyr, and beyond.
* Right-click the CMakeLists.txt file in the desired pellet/magnet_module directory; select
  Load CMake Project.
* In Settings->Build, Execution, Deployment->CMake, for the profile, specify the
  build directory: build/<module>.
  * All other default values are OK.
* Build the target 'zephyr_final'.
* Flash the application per _Flashing an Application_.

##  CMSIS DSP

The FFT library relies on the CMSIS DSP support under zephyr. The following files
were updated:
* west.yml - In the name-allow list, added `- cmsis-dsp`
* prj.conf - Added, _near the top of the file_:
```
CONFIG_CMSIS_DSP=y
CONFIG_CMSIS_DSP_COMPLEXMATH=y
CONFIG_CMSIS_DSP_TRANSFORM=y
```
* In the KConfig file for FFT, _absolutely do not add_ dependencies on CMSIS_DSP or
similar. For some reason, it excludes the FFT library files if you do that.
* There is no need to link the CMSIS DSP library in the CMakeLists.txt. In fact,
_don't do that_ either.

## Building the Pellet Module for a NUCLEO-G474RE

The pellet module firmware also builds for ST's NUCLEO-G474RE development board,
which carries the same STM32G474RE. After initializing the West workspace as described
above, run this command inside the container:

```bash
cd /auto-trainer-hardware/firmware/pellet_module
west build -p --sysbuild -b nucleo_g474re
```

The resulting application images are:

* `firmware/pellet_module/build/pellet_module/zephyr/zephyr.hex`
* `firmware/pellet_module/build/pellet_module/zephyr/zephyr.bin`

This is a bring-up target, not a second product configuration. Two differences are
worth knowing:

* **No bootloader.** The dev board has 512K of internal flash and no external SPI-NOR
  to hold a second image slot, and two slots large enough for this firmware do not fit.
  `sysbuild_nucleo_g474re.conf` selects `SB_CONFIG_BOOTLOADER_NONE`, so the JerryCAN
  bootloader commands are absent and the image is flashed directly rather than signed.
* **Pins are stand-ins.** Every peripheral instance, DMAMUX request line and timer
  matches the pellet module board, but the pins are remapped onto ones the Nucleo
  brings out on its Arduino and ST-morpho headers. `boards/nucleo_g474re.overlay`
  lists the full map. Nothing is wired to real hardware, and the board has no CAN
  transceiver, so FDCAN1 (PA11/PA12) needs one attached to talk to anything.

# Building on the Host without the Docker Image

This project uses Zephyr RTOS 3.7.0, West for project management, and Zephyr SDK
0.16.8. The commands below install the same minimal SDK and ARM toolchain used by the
Docker image. `$HOME` is one of Zephyr's default SDK search locations, and the `-c`
option also registers the SDK in the current user's CMake package registry.

On a 64-bit x86 Linux host, run:

```bash
cd "$HOME"
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/zephyr-sdk-0.16.8_linux-x86_64_minimal.tar.xz
tar xf zephyr-sdk-0.16.8_linux-x86_64_minimal.tar.xz
rm zephyr-sdk-0.16.8_linux-x86_64_minimal.tar.xz
cd zephyr-sdk-0.16.8
./setup.sh -t arm-zephyr-eabi -c -h
```

On a 64-bit ARM Linux host, run:

```bash
cd "$HOME"
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/zephyr-sdk-0.16.8_linux-aarch64_minimal.tar.xz
tar xf zephyr-sdk-0.16.8_linux-aarch64_minimal.tar.xz
rm zephyr-sdk-0.16.8_linux-aarch64_minimal.tar.xz
cd zephyr-sdk-0.16.8
./setup.sh -t arm-zephyr-eabi -c -h
```

Run `setup.sh` as your normal user so CMake registration is written to that user's
package registry. The [Zephyr 3.7.0 Getting Started Guide](https://docs.zephyrproject.org/3.7.0/develop/getting_started/index.html)
has additional operating-system dependency, SDK verification, and flashing setup
information.

## Python Virtual Environment

Create and activate a virtual environment in the cloned repository, then install West
and this project's Python requirements:

```bash
cd "$HOME/auto-trainer-hardware"
python3 -m venv .venv
source .venv/bin/activate
python -m pip install west
python -m pip install -r firmware/requirements.txt
```

## Initializing the Project Workspace

Initialize the workspace and install the device pack used to flash the STM32G4:

```bash
cd firmware
west init -l
west update
west config build.sysbuild true
pyocd pack install stm32g4
```

You can now use the native-board or NUCLEO-G474RE build commands in this document,
replacing the `/auto-trainer-hardware` container path with the path to your local clone.
