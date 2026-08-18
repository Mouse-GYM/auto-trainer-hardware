# Autotrainer Firmware

## Setup

This project is built on the Zephyr RTOS. It uses the West tool in order to handle various project tasks,
such as building and flashing.

See the Zephyr Getting Started Guide for how to install the various tools needed for developers
https://docs.zephyrproject.org/latest/develop/getting_started/index.html

It is recommended to do `west config build.sysbuild True` to have West default to using sysbuild at all times.

### Python Virtual Environment

You will also want to create a virtual environment for this build:

```bash
cd <parent>/autotrainer
python -m venv .venv

# Then activate the virtual environment
source .venv/bin/activate
```

### Initializing Project Workspace

Once West is installed, initialize the project workspace:

```bash
cd <parent>/autotrainer
west init -l
west update
```

Install the python requirements:
```bash
cd firmware
pip install -r requirements.txt
```

And install tools to support flashing the firmware:
```bash
pyocd pack install stm32g4
```

## Building an Application

For each board (pellet and magnet), there are two firmware images that need to be built: 
* MCUBoot
* Application

West implements sysbuild which coordinates building both of these projects for us.

### Building Pellet Module

The first time building the project (or if you ever delete the `build` directory), run this command:

```bash
cd firmware/pellet_module
west build --sysbuild --board cerebellumlab_pellet_module -p
```

Thereafter, this command is sufficient:

```bash
west build
```

The resulting file is: `firmware/pellet_module/build/pellet_module/zephyr/zephyr.signed.bin`

### Building Magnet Module

The first time building the project (or if you ever delete the `build` directory), run this command:

```bash
cd firmware/magnet_module
west build --sysbuild --board cerebellumlab_magnet_module -p
```

Thereafter, this command is sufficient:

```bash
west build
```

The resulting file is: `firmware/magnet_module/build/magnet_module/zephyr/zephyr.signed.bin`

### Building the Pellet Module for a NUCLEO-G474RE

The pellet module firmware also builds for ST's NUCLEO-G474RE dev board, which carries
the same STM32G474RE:

```bash
cd firmware/pellet_module
west build --sysbuild --board nucleo_g474re -p
```

The resulting file is: `firmware/pellet_module/build/pellet_module/zephyr/zephyr.bin`

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

## Flashing an Application

To flash a module, cd to the desired application directory and issue:

```bash
west flash
```

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

# CU Python Application Code

## Setup

The code resides at: https://github.com/Mouse-GYM/auto-trainer

If the python code is run on the Jetson, there is no need to create a running
environment on the development machine. However, it is more convenient to run
the application code on the development machine.

For the work under the purview of firmware, the camera and other device support is 
not required for development against the firmware.

This information is pulled from the Python repo README and may be incomplete:

### Installation for Development Environment
* Install support packages

```bash
sudo apt-get install libhdf5-serial-dev
sudo apt-get install libxcb-cursor0 
````

* See online documentation for installation of anaconda (miniconda is sufficient for development)
* Create the virtual environment
```bash
  conda create --name <name> python=3.8
```

* Initialize the virtual environment
```shell
conda init
```

* Enable the environment
```bash
conda activate <name>
```

* Install supporting python packages
```shell
cd <parnet>/auto-trainer
pip install -r requirements.txt
pip install -r requirements-test.txt
```

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

## Device Python Package

In the auto-trainer Python repository, all hardware interaction software is 
in the auto-trainer-device. In that directory, there may be (unless its been deprecated)
references to a serial device (original device); that code can be generally ignored.

In that part of the source code, there are two files of interest:
* can_interface.py - It's an extra layer between the libjerrycan library and the rest
of the application. It transforms data sets from a libjerrycan-specific structure to
a more generic set of classes, and provides ease-of-use methods to command the
different aspects of the hardware (e.g. run a motor). It does it in a way that the
source/destination of the data/commands is agnostic - the auto trainer device controls
appears as a single piece of hardware.
* can_device.py - This layer conforms to a send message/receive data protocol that 
abstracts the interface to the underlying hardware. This allows easy swap-out of legacy
hardware with the updated hardware.

## Supporting Applications

To date, only 3 applications have been supported by the firmware development team:

* CAN console - A console application that allows quick-and-easy access to
the functionality provided by can_device.py. It's a console menu-driven application.
  * It resides in scripts/can_console.py
* HEAD Fix U/I - A U/I front end that provides most of the same support that CAN 
console does, only with a U/I front end, and only those items supported by the 
magnet module (exception: Tone generation).
  * It resides in tools/head_fix.
* Pellet U/I - A U/I front end that provides most of the same support that CAN console 
does, only with a U/I front end, and only those items supported by the pellet module.
  * It resides in tools/pellet_delivery.
  
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