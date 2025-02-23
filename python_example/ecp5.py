import ice_flasher

flasher = ice_flasher.IceFlasher()

pins = {
        'sck':10,
        'mosi':11,
        'cs':12,
        'miso':13,
        }

flasher.spi_configure(pins['sck'], pins['cs'], pins['mosi'],pins['miso'], 1)
flasher.gpio_set_direction(pins['cs'], True)
flasher.gpio_put(pins['cs'], True)

cmd = bytearray(8)
cmd[0] = 0b11100000  # read_id

# From ECP5 and ECP5-5G sysCONFIG User Guide, Table B.5. ECP5 and ECP5-5G Device ID:
# IDCODE for LFE5U-85 should be 0x41113043
ret = flasher.spi_rxtx(cmd)
print(ret)
