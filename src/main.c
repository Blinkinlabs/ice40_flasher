/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <hardware/gpio.h>
#include <hardware/adc.h>

#include "bsp/board.h"
#include "tusb.h"

/* This example demonstrate HID Generic raw Input & Output.
 * It will receive data from Host (In endpoint) and echo back (Out endpoint).
 * HID Report descriptor use vendor for usage page (using template TUD_HID_REPORT_DESC_GENERIC_INOUT)
 *
 * There are 2 ways to test the sketch
 * 1. Using nodejs
 * - Install nodejs and npm to your PC
 *
 * - Install excellent node-hid (https://github.com/node-hid/node-hid) by
 *   $ npm install node-hid
 *
 * - Run provided hid test script
 *   $ node hid_test.js
 *
 * 2. Using python
 * - Install `hid` package (https://pypi.org/project/hid/) by
 *   $ pip install hid
 *
 * - hid package replies on hidapi (https://github.com/libusb/hidapi) for backend,
 *   which already available in Linux. However on windows, you may need to download its dlls from their release page and
 *   copy it over to folder where python is installed.
 *
 * - Run provided hid test script to send and receive data to this device.
 *   $ python3 hid_test.py
 */

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);

void pins_init();

/*------------- MAIN -------------*/
int main(void)
{
    board_init();
    tusb_init();

    pins_init();

    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);

    while (1)
    {
        tud_task(); // tinyusb device task
        led_blinking_task();
    }

    return 0;
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) itf;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

#define PIN_COUNT 23
#define PIN_MASK ((1<<PIN_COUNT) - 1)
//#define PIN_MASK (0xFFFFFFFF)

void pins_init() {
    gpio_init_mask(PIN_MASK);
}

void spi_bitbang(const uint8_t sck_pin,
        const uint8_t mosi_pin,
        const uint8_t miso_pin,
        const uint32_t bit_count,
        uint8_t const* buf_out,
        uint8_t* buf_in) {
    memset(buf_in, 0, (bit_count + 7)/8);

    for(uint32_t bit_index = 0; bit_index < bit_count; bit_index++) {
        const int byte_index = bit_index / 8;
        const uint8_t bit_offset = bit_index % 8;

        gpio_put(mosi_pin, (buf_out[byte_index] << bit_offset) & 0x80);
        gpio_put(sck_pin, true);
        //sleep_us(1);

        if(gpio_get(miso_pin)) {
            buf_in[byte_index] |= (1<<(7-bit_offset));
        }

        gpio_put(sck_pin, false);
        //sleep_us(1);
    }
}

uint32_t adc_sample_input(uint8_t input) {
    adc_select_input(input);
 
    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    const uint32_t conversion_factor = 3300000 / (1 << 12); // microvolts per sample
    uint16_t result = adc_read();

    return result*conversion_factor;
}


uint32_t read_uint32(uint8_t const* buffer) {
    const uint32_t val = 
          (*(buffer + 0) << 24)
        + (*(buffer + 1) << 16)
        + (*(buffer + 2) << 8)
        + (*(buffer + 3) << 0);
    return val;
}

void write_uint32(uint32_t val, uint8_t* buffer) {
    buffer[0] = ((val >> 24) & 0xFF);
    buffer[1] = ((val >> 16) & 0xFF);
    buffer[2] = ((val >> 8) & 0xFF);
    buffer[3] = ((val >> 0) & 0xFF);
}



// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  // This example doesn't use multiple report and report ID
  (void) itf;
  (void) report_id;
  (void) report_type;

  switch(buffer[0]) {
    case 0x00:      // set LED
        board_led_write(buffer[1] & 0x01);
        break;

    case 0x10:      // pin direction
        gpio_set_dir_masked(
            read_uint32(&buffer[1]) & PIN_MASK,
            read_uint32(&buffer[5]) & PIN_MASK
            );
        break;
    case 0x20:      // set pin values
        gpio_put_masked(
            read_uint32(&buffer[1]) & PIN_MASK,
            read_uint32(&buffer[5]) & PIN_MASK
            );
        break;
    case 0x30:      // read pin values
        {
            const uint32_t pin_values = gpio_get_all() & PIN_MASK;
            uint8_t ret_buffer[1+4];
            ret_buffer[0] = 0x30;
            write_uint32(pin_values, &ret_buffer[1]);
            tud_hid_report(0, ret_buffer, sizeof(ret_buffer));
        }
        break;
    case 0x40:      // bitbang spi
        {
            uint8_t ret_buffer[64];
            ret_buffer[0] = 0x40;
            write_uint32(read_uint32(&buffer[4]), &ret_buffer[1]);
            
            spi_bitbang(
                buffer[1],
                buffer[2],
                buffer[3],
                read_uint32(&buffer[4]),
                &buffer[8],
                &ret_buffer[5]
                );

            tud_hid_report(0, ret_buffer, sizeof(ret_buffer));
        }
    case 0x50:      // Read ADCs
        {
            uint8_t ret_buffer[1+3*4];
            ret_buffer[0] = 0x50;
            write_uint32(adc_sample_input(0), &ret_buffer[1]);
            write_uint32(adc_sample_input(1), &ret_buffer[5]);
            write_uint32(adc_sample_input(2), &ret_buffer[9]);
            tud_hid_report(0, ret_buffer, sizeof(ret_buffer));
        }
    }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
