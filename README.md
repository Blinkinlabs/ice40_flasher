# Ice40 programmer

This firmware allows a Raspberry Pi Pico (or any RP2040) to work as a programmer for the lattice ice40 parts.

It has been integrated into icestorm:
https://github.com/Blinkinlabs/icestorm/commits/interfaces

Advantages:
* Cheap: RPi Pico boards are currently EUR4, FT232H boards are closer to EUR15
* Available: As of summer '22, FT232H boards and chips are in short supply; Pico boards are still readily available
* Flexible: Any GPIO-capable pins on the pico can be used for programming. This allow for example multiple ice40 parts to be programmed from a single Pico.

## Usage

First, program your RPi Pico using the included binary 'main.u2f'. To do so, disconnect the pico from your computer, press down the bootloader button, then plug the pico back in. The computer should detect it as a memory device. Copy the main.u2f file into the root directory of this memory device. This will program the Pico. Once completed, the pico should restart and present itself as a USB HID device.

To use it with icestorm, the 'iceprog' utility will need to be built from the above fork. Known issues:

* This version of iceprog is hard-coded to use the pico as a programmer; it should have a command-line switch to choose the correct programmer
* USB HID has limited bandwidth, and it takes approximately 20 seconds to reprogram the flash (vs 5 seconds for FTDI).
* SRAM programming mode (-S) is untested

TODO: Suggested wiring diagram
TODO: Permissions

## Command set

The firmware implements 6 commands over a single HID endpoint. The commands are:

| ID   | Command length | Return length | Description |
| ---  | ---            | ---           |--- |
| 0x00 | 2              | 0             | Set LED state |
| 0x10 | 9              | 0             | Set pin directions | 
| 0x20 | 9              | 0             | Set pin values |
| 0x30 | 1              | 5             | Read pin values |
| 0x40 | 9+n            | 1+n           | Bitbang SPI |
| 0x50 | 1              | 13            | Read ADCs |

### Set LED state

This command turns the on-board LED on or off

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x00   | 1      | ID (0x00)   |
| 0x01   | 1      | LED state: 0x00=off, 0x01=on |

Note: The current firmware implements a 'blink' pattern that overrides this setting.

### Set pin directions

This command is used to set pin directions. The first field is a mask of pins to update, and the second is a bitmap of the resulting states. Any pin that has a bit set in the mask will be updated.

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x00   | 1      | ID (0x10)   |
| 0x01   | 4      | uint32: Pin mask (1=set direction) |
| 0x05   | 4      | uint32: Pin direction (1=output, 0=input) |

The firmware does not send a response packet.

### Set pin pullup/pulldown resistors

This command is used to set pin pullup/pulldown resistors. The first field is a mask of pins to update, the second is the pull-up states, and the third is the pull-down states. Any pin that has a bit set in the mask will be updated.

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x00   | 1      | ID (0x12)   |
| 0x01   | 4      | uint32: Pin mask (1=set direction) |
| 0x05   | 4      | uint32: Pin pullups (1=enable, 0=disable) |
| 0x09   | 4      | uint32: Pin pulldowns (1=enable, 0=disable) |

The firmware does not send a response packet.

### Set pin values

This command is used to set the value of output pins. The first field is a mask of pins to update, and the second is a bitmap of new output values to apply. Any pin that has a bit set in the mask will be updated.

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x00   | 1      | ID (0x20)   |
| 0x01   | 4      | uint32: Pin mask (1=set direction) |
| 0x05   | 4      | uint32: Pin value (1=high, 0=low) |

The firmware does not send a response packet.

### Read pin values

This command is used to read the value of all pins. Note that pins which are confgured as outputs will report their current output setting.

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x00   | 1      | ID (0x30)   |

Response packet:

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x00   | 1      | ID (0x30)   |
| 0x05   | 4      | uint32: Pin values (1=high, 0=low) |

### Bitbang SPI

This command is used to emulate a SPI protocol by bitbanging GPIO pins. The achieved clock speed is around 2 MHz. Note that this routine does not emulate a CS signal. If needed, that must be emulated separately by using the set pin values command to toggle a separate GPIO high and low.

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x00   | 1      | ID (0x40)   |
| 0x01   | 1      | GPIO pin number to use for SCK |
| 0x02   | 1      | GPIO pin number to use for MOSI |
| 0x03   | 1      | GPIO pin number to use for MISO |
| 0x04   | 4      | Bits to transfer |
| 0x08   | n      | output data (1-448 bits = 1-56 bytes). Any remainder bits should be left-aligned. |

Response packet:

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x00   | 1      | ID (0x40)   |
| 0x01   | 4      | Bits transferred |
| 0x05   | n      | input data (1-448 bits = 1-56 bytes). Any remainder bits are left-aligned. |

### Read ADCs

This command is used to read the analog value of analog input pins 0-2. Each value is returned as a uint32_t value representing the reading in microvolts.

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x00   | 1      | ID (0x50)   |

Response packet:

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x00   | 1      | ID (0x50)   |
| 0x01   | 4      | ADC channel 0 value, in microvolts |
| 0x05   | 4      | ADC channel 1 value, in microvolts |
| 0x09   | 4      | ADC channel 2 value, in microvolts |

## Building the firmware

First, install the [Rasberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk.git):

    sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
    cd ~
    git clone https://github.com/raspberrypi/pico-sdk.git

Then, clone and build this repository:

    cd ~
    git clone https://github.com/Blinkinlabs/ice40_flasher
    cd ice40_flasher
    export PICO_SDK_PATH=~/pico/pico-sdk
    mkdir build
    cd build
    cmake ..

Finally, load the firmware onto the Pico using the instructions in the [usage](https://github.com/Blinkinlabs/ice40_flasher#usage) section.
