import ice_flasher
import struct
import time
import argparse

commands = {
        'READ_ID':              {'opcode':0xE0,'class':'A','ret_len':4},
        'LSC_READ_STATUS':      {'opcode':0x3C,'class':'A','ret_len':4},
        'LSC_CHECK_BUSY':       {'opcode':0xF0,'class':'A','ret_len':1},
        'LSC_REFRESH':          {'opcode':0x79,'class':'D'},
        'ISC_ENABLE':           {'opcode':0xC6,'class':'C'},
        'ISC_DISABLE':          {'opcode':0x26,'class':'C'},
        'LSC_BITSTREAM_BURST':  {'opcode':0x7A,'class':'B'},
        }

def wait_busy(fl, timeout=4):
    start_time = time.time()
    while time.time() - start_time < timeout:
        ret = run_command(fl,'LSC_CHECK_BUSY')
        if ret == b'\x00':
            return

    raise ValueError('Timeout!')


def run_command(fl, name, tx_data=None):
    command = commands[name]

    if command['class'] == 'A':
        buf = bytearray(1+3+command['ret_len'])
        buf[0] = command['opcode']
        ret = fl.spi_rxtx(buf)
        return ret[4:]

    elif command['class'] == 'B':
        buf = bytearray(1+3)
        buf[0] = command['opcode']
        buf.extend(tx_data)
        fl.spi_rxtx(buf)

    elif command['class'] == 'C':
        buf = bytearray(1+3)
        buf[0] = command['opcode']
        fl.spi_rxtx(buf)

    elif command['class'] == 'D':
        buf = bytearray(1+3)
        buf[0] = command['opcode']
        fl.spi_rxtx(buf)
        wait_busy(fl)

    else:
        raise ValueError(f"Invalid command class:{command['class']}")


pins = {
        'SCK':10,
        'MOSI':11,
        'CS':12,
        'MISO':13,
        'PROGRAMN':14,
        }


def load_image(filename):
    flasher = ice_flasher.IceFlasher()
    
    flasher.spi_configure(pins['SCK'], pins['CS'], pins['MOSI'],pins['MISO'], 10)
    
    # From: 6.2.4. Slave SPI Configuration Flow Diagram:
    
    flasher.gpio_put(pins['PROGRAMN'], False)       # Step 0: reset
    flasher.gpio_set_direction(pins['PROGRAMN'], True)
    flasher.gpio_set_direction(pins['PROGRAMN'], False)
    
    time.sleep(0.05)                                # Step 1
    
    # From ECP5 and ECP5-5G sysCONFIG User Guide, Table B.5. ECP5 and ECP5-5G Device ID:
    # IDCODE for LFE5U-85 should be 0x41113043
    chip_id = run_command(flasher,'READ_ID')        # Step 2-3
    if struct.unpack('>I', chip_id)[0] != 0x41113043:
        raise ValueError(f"read bad id: {hex(struct.unpack('>I', chip_id)[0])}")
    
    run_command(flasher,'ISC_ENABLE')               # Step 4
    
    with open(filename, 'rb') as f:              # Steps 5-6
        run_command(flasher,'LSC_BITSTREAM_BURST',tx_data=f.read())
    
    status = run_command(flasher,'LSC_READ_STATUS') # Step 7
    print('Status:', ' '.join([f"{x:02X}" for x in status]))
    
    run_command(flasher,'ISC_DISABLE')              # Step 8

if __name__=='__main__':
    parser = argparse.ArgumentParser(
        prog='load_ecp5',
        description='Configure an ECP5 using an rp2350')
    
    parser.add_argument('filename')
    args = parser.parse_args()

    load_image(args.filename)
