# Electronic Cats SAM D|L|C Core for Arduino

[![Build Status](https://github.com/ElectronicCats/ArduinoCore-samd/workflows/Build/badge.svg)](https://github.com/ElectronicCats/ArduinoCore-samd/actions)

<a href="https://github.com/sponsors/ElectronicCats">
  <img src="https://electroniccats.com/wp-content/uploads/2020/07/Badge_GHS.png" height="104" />
</a>

The Electronic Cats SAM D|L|C Core for Arduino is a fork from MattairTech/ArduinoCore-samd
on GitHub, which will be used to maintain Arduino support for Electronic Cats boards.

### Microcontroller supported

- SAMD21
- SAML21
- SAMC21
- SAMD11
- SAMR34 or RAK4260
- SAMD51

### SAMD (ARM Cortex-M0+) Boards by Electronic Cats
* Supports all board by Electronic Cats
* Supports four clock sources (two crystals, internal oscillator, and USB calibrated).
* Supports the bootloaders UF2. Available at [bootloaders UF2 SAMD21 repository](https://github.com/ElectronicCats/uf2-samd21).
* USB CDC Bootloader with optional SDCard support. See [bootloaders/zero/README.md](https://github.com/ElectronicCats/ArduinoCore-samd/tree/master/bootloaders/zero/README.md).

## Installation Instruction "Electronic Cats" support

**Instruction Details**

More details about Installation Instruction visit our [Wiki](https://github.com/ElectronicCats/ArduinoCore-samd/wiki/Installation-Instruction)

**Simple Install**

To add board support for our products, start Arduino and open the Preferences window (**File** > **Preferences**). Now copy and paste the following URL into the 'Additional Boards Manager URLs' input field:

	https://electroniccats.github.io/Arduino_Boards_Index/package_electroniccats_index.json


## Differences Between Electronic Cats and Arduino Cores

* Communications interfaces are mostly unchanged, including USB
* All pins have high drive strength enabled by default
* All pins (digital and analog) setup in STARTUP mode (enable INEN and set default pull direction to pullup (pullup will not be enabled))
* INEN enabled for both input and output (but not analog)
* pinPeripheral now handles disabling the DAC (if active). Note that on the L21, the DAC output would
  interfere with other peripherals if left enabled, even if the anaolog peripheral is not selected.
* Pull resistors enabled only if pin attributes allow and only if pin is not configured as output.
* Pull direction (pullup or pulldown) is now set with pinMode only (defaults to pullup if pinMode never called).
* At least on the L21, pin A31 must be set as an input. It is possible that debugger probe detection is being falsely
  detected (even with a pullup on A31 (SWCLK)), which would change the peripheral mux of A31 to COM.
  This might not normally be a problem, but one strange effect is that Serial2 loses characters if pin A31 is not set as INPUT.
  So, the startup code calls pinMode(31, INPUT).
* Todo: Table summarizing which core files are modified and by how much
* Todo: List changes due to adding/changing features vs porting to new chip
* Added support for SAMR34
* Added support for Electronic Cats Boards
* Remove Mattairtech Boards
* Update with Arduino Core official fix
* Remove menu for SPI, Serial and I2C interfaces
* Thanks Mattairtech!

[Variant Compliance Changelog](https://github.com/ElectronicCats/ArduinoCore-samd/blob/master/VARIANT_COMPLIANCE_CHANGELOG)

### API Core

 All Features and Configuration for the Core is available in [APICore.md](https://github.com/ElectronicCats/ArduinoCore-samd/blob/master/APICore.md)

### Features

* SAM-BA USB CDC and UART interfaces with optional terminal mode
* SD Card interface (both USB CDC and SD Card support fits in 8KB)
* Four different clock sources (two external crystals and two internal oscillator options)
* Arduino IDE auto-reset and double-tap reset button support
* Arduino extended commands for faster firmware loading
* Supports the SAM D51, D21, L21, C21, R34 and D11.
* Bossac command line utility for Windows, Linux, and OS X

The bootloader can be started by:

   * Tapping reset twice in quick succession (BOOT_DOUBLE_TAP).
   * Holding down button A (BOOT_LOAD_PIN) while powering up.
   * Clicking 'Upload Sketch' in the Arduino IDE, which will automatically start the bootloader (when CDC is enabled).
   * If the application (sketch) area is blank, the bootloader will run.

Otherwise, it jumps to application and starts execution from there. The LED will PWM fade during bootloader execution.

### Special Libraries for this Core

- [ArduinoLowPower fork of Arduino](https://github.com/ElectronicCats/ArduinoLowPower)
- [RTCZero fork of Arduino](https://github.com/ElectronicCats/RTCZero)

### Bossac

Bossac is a command line utility for uploading firmware to SAM-BA bootloaders. It runs on Windows. Linux, and OS X.
It is used by Arduino to upload firmware to SAM and SAMD boards. The version described here adds to the
Arduino version (https://github.com/shumatech/BOSSA, Arduino branch), which in turn is a fork from the original
Bossa (http://www.shumatech.com/web/products/bossa). It adds support for more SAM chips (D51, D21, L21, C21, and D11),
support for four clock sources, and firmware loading from a MicroSD card.


### Bootloader Firmware Installation

[Instruction for Bootloader Firmware Installation](https://github.com/ElectronicCats/ArduinoCore-samd/wiki/Bootloader-Firmware-Installation)

### Bootloader Binaries

The [bootloaders UF2 SAMD21 repository](https://github.com/ElectronicCats/uf2-samd21) repository contains the UF2
bootloaders from the 'Electronic Cats SAM D|L|C Core for Arduino', which is
available at https://github.com/ElectronicCats/uf2-samd21.


## Contributions

Contributions are always welcome. The preferred way to receive code cotribution is by submitting a Pull Request on github.


## License and Credits

This core has been developed by Arduino LLC in collaboration with Atmel.
This fork developed Electronic Cats SAPI de CV.
based in developed by Justin Mattair of MattairTech LLC.


```
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2017-2018 MattairTech LLC. All right reserved.
  Copyright (c) 2018-2021 Electronic Cats SAPI de CV. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
```

### Petit FatFS

Petit FatFs module is an open source software to implement FAT file system to
small embedded systems. This is a free software and is opened for education,
research and commercial developments under license policy of following trems.

Copyright (C) 2014, ChaN, all right reserved.

* The Petit FatFs module is a free software and there is NO WARRANTY.
* No restriction on use. You can use, modify and redistribute it for
  personal, non-profit or commercial use UNDER YOUR RESPONSIBILITY.
* Redistributions of source code must retain the above copyright notice.
