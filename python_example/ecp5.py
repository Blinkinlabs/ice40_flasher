import ice_flasher
import struct
import time

def class_a(fl, opcode, ret_len):
    cmd = bytearray(1+3+ret_len)
    cmd[0] = opcode
    ret = fl.spi_rxtx(cmd)
    return ret[4:]

class_a_commands = {
    'READ_ID':0xE0,
    'LSC_READ_STATUS':0x3C,
    }

def class_c(fl, opcode):
    cmd = bytearray(1+3)
    cmd[0] = opcode
    fl.spi_rxtx(cmd)

class_c_commands = {
    'ISC_ENABLE':0xC6,
    'ISC_DISABLE':0x26,
    'LSC_BITSTREAM_BURST':0x7A,
    }



flasher = ice_flasher.IceFlasher()

pins = {
        'SCK':10,
        'MOSI':11,
        'CS':12,
        'MISO':13,
        'PROGRAMN':14,
        }

flasher.spi_configure(pins['SCK'], pins['CS'], pins['MOSI'],pins['MISO'], 1)
flasher.gpio_set_direction(pins['CS'], True)
flasher.gpio_put(pins['CS'], True)

#flasher.gpio_put(pins['PROGRAMN'], False)
#flasher.gpio_set_direction(pins['PROGRAMN'], True)
#time.sleep(.1)
#flasher.gpio_set_direction(pins['PROGRAMN'], False)

#class_c(flasher, 0x79) # refresh
#time.sleep(2)

# From: 6.2.4. Slave SPI Configuration Flow Diagram:

### 2-3 Read ID
# From ECP5 and ECP5-5G sysCONFIG User Guide, Table B.5. ECP5 and ECP5-5G Device ID:
# IDCODE for LFE5U-85 should be 0x41113043
chip_id = class_a(flasher, class_a_commands['READ_ID'], 4)
if struct.unpack('>I', chip_id)[0] != 0x41113043:
    raise ValueError(f"read bad id: {hex(struct.unpack('>I', chip_id)[0])}")

### 4 ISC_ENABLE
class_c(flasher, class_c_commands['ISC_ENABLE'])

### 5 LSC_BITSTREAM_BURST
class_c(flasher, class_c_commands['LSC_BITSTREAM_BURST'])

### 6 Clock in bitstream
flasher.gpio_put(pins['CS'], False)

with open('blink.bit', 'rb') as f:
    flasher.spi_rxtx(f.read(), False)

flasher.gpio_put(pins['CS'], True)

### 7 Check status register
status = class_a(flasher, class_a_commands['LSC_READ_STATUS'], 4)
print('Status:', ' '.join([f"{x:02X}" for x in status]))

### 8 Clock in ISC_DISABLE
class_c(flasher, class_c_commands['ISC_DISABLE'])

