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
* SRAM programming mode (-S) is untested

TODO: Suggested wiring diagram

TODO: Permissions

## Command set

Commands are sent to the device as [control transfers](https://www-user.tu-chemnitz.de/~heha/hsn/chm/usb.chm/usb4.htm#Control). The bRequest field is used to select the commmand, and any configuration data associated with the command 


| Command | Direction | bRequest | wValue | wIndex | wLength | Description |
| ---  | ---  | --- |--- | --- |--- |--- |
| PIN_DIRECTION_SET  | OUT | 0x10  | 0 | 0 | 8 | Set GPIO pin directions |
| PULLUPS_SET        | OUT | 0x12  | 0 | 0 | 12 | Set GPIO pull-ups |
| PIN_VALUES_SET     | OUT | 0x20 | 0 | 0 | 4 | Set GPIO output values |
| PIN_VALUES_GET     | IN  | 0x30 | 0 | 0 | 4 | Get GPIO input values |
| SPI_BITBANG_CS     | OUT | 0x41 | 0 | 0 | 4+n | Write SPI transaction, toggling CS pin |
| SPI_BITBANG_CS     | IN  | 0x41 | 0 | 0 | n | Read data returned from previous SPI transaction |
| SPI_BITBANG_NO_CS  | OUT | 0x42 | 0 | 0 | 4+n | Write SPI transaction without toggling CS pin |
| SPI_BITBANG_NO_CS  | IN  | 0x42 | 0 | 0 | n | Read data returned from previous SPI transaction |
| SPI_PINS_SET       | OUT | 0x43 | 0 | 0 | 5 | Configure SPI pins |
| SPI_CLKOUT         | OUT | 0x44 | 0 | 0 | ? | Toggle the clock pin |
| ADC_READ           | IN  | 0x50 | 0 | 0 | ? | Read ADC inputs |
| BOOTLOADER         | OUT | 0xFF | 0 | 0 | 0 | Jump to bootloader mode |

Additionally, the device supports an additional control transfer to support driver assignment on Windows:

| Command | Direction | bRequest | wValue | wIndex | wLength | Description |
| ---  | ---  | --- |--- | --- |--- |--- |
| MS_DESCRIPTOR      | IN | 0xF8 | 0 | 0 | x | Get a Microsoft OS compatible descriptor |

### Set pin directions

This command is used to set pin directions. The first field is a mask of pins to update, and the second is a bitmap of the resulting states. Any pin that has a bit set in the mask will be updated.

Data packet format:

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x00   | 4      | uint32: Pin mask (1=set direction) |
| 0x04   | 4      | uint32: Pin direction (1=output, 0=input) |

### Set pin pullup/pulldown resistors

This command is used to set pin pullup/pulldown resistors. The first field is a mask of pins to update, the second is the pull-up states, and the third is the pull-down states. Any pin that has a bit set in the mask will be updated.

Data packet format:

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x00   | 4      | uint32: Pin mask (1=set direction) |
| 0x04   | 4      | uint32: Pin pullups (1=enable, 0=disable) |
| 0x08   | 4      | uint32: Pin pulldowns (1=enable, 0=disable) |

### Set pin values

This command is used to set the value of output pins. The first field is a mask of pins to update, and the second is a bitmap of new output values to apply. Any pin that has a bit set in the mask will be updated.

Data packet format:

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x00   | 4      | uint32: Pin mask (1=set direction) |
| 0x04   | 4      | uint32: Pin value (1=high, 0=low) |

### Read pin values

This command is used to read the value of all pins. Note that pins which are confgured as outputs will report their current output setting.

Data packet format:

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x05   | 4      | uint32: Pin values (1=high, 0=low) |

### Write SPI transaction, toggling CS pin

This command is used to send data over the pins currently configured as the SPI interface. This command performs a full-duplex read/write operation, and stores the read data in a buffer. To retrieve the data read during this operation, issue a 

Data packet format:

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x04   | 4      | Bytes to transfer |
| 0x08   | 1-2040 | SPI data to transfer |

### Get data read during previous SPI transaction

This command is used to retrieve any data transferred during the previous SPI transaction.

Data packet format:

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x00   | 1-2040 | SPI data to transfer |

### SPI clock out

This command is used to toggle the SPI clock pin, but doesn't transfer any data.

Data packet format:

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x00   | 4 | Number of SPI bytes to clock |

### Set SPI configuration

This command is used to configure the SPI engine.

Data packet format:

| Offset | Length | Description |
| ---    | ---    | ---         |
| 0x00   | 1      | GPIO pin number for SCK |
| 0x01   | 1      | GPIO pin number for CS |
| 0x02   | 1      | GPIO pin number for MOSI |
| 0x03   | 1      | GPIO pin number for MISO |
| 0x04   | 1      | SPI clock frequency, in MHz |


### Read ADCs xxx

This command is used to read the analog value of analog input pins 0-2. Each value is returned as a uint32_t value representing the reading in microvolts.

Data packet format:

| Offset | Length | Description |
| ---    | ---    | ---         | 
| 0x01   | 4      | ADC channel 0 value, in microvolts |
| 0x05   | 4      | ADC channel 1 value, in microvolts |
| 0x09   | 4      | ADC channel 2 value, in microvolts |

### Bootloader

This command is used to put the device in bootloader mode. No data is sent during this 

## Building the firmware

First, install the [Rasberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk.git):

    sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
    cd ~
    git clone https://github.com/raspberrypi/pico-sdk.git
    cd pico-sdk
    git submodule update --init

Then, clone and build this repository:

    cd ~
    git clone https://github.com/Blinkinlabs/ice40_flasher
    cd ice40_flasher
    export PICO_SDK_PATH=~/pico-sdk
    mkdir build
    cd build
    cmake ..
    make

Finally, load the firmware onto the Pico using the instructions in the [usage](https://github.com/Blinkinlabs/ice40_flasher#usage) section.
