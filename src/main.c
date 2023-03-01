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
#include "pico/bootrom.h"
#include "bsp/board.h"
#include "tusb.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum
{
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED = 1000,
    BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

typedef struct
{
    uint8_t sck;
    uint8_t cs;
    uint8_t mosi;
    uint8_t miso;
} spi_pins_t;

static spi_pins_t spi_pins;

void led_blinking_task(void);

void pins_init();

/*------------- MAIN -------------*/
int main(void)
{
    board_init();
    tusb_init();

    pins_init();

    adc_init();

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
    (void)remote_wakeup_en;
    blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    blink_interval_ms = BLINK_MOUNTED;
}

#define PIN_COUNT 32 // PIN_MASK is 32 bits, so that's the max number of pins available
#define PIN_MASK (0b00011100011111111111111111111111)
// #define PIN_MASK ((1<<PIN_COUNT) - 1)
// #define PIN_MASK (0xFFFFFFFF)

void pins_init()
{
    gpio_init_mask(PIN_MASK);
}

//! @brief Set the pull-up state for the gpios specified in the pinmask
void set_pulls_masked(
    const uint32_t mask,
    const uint32_t ups,
    const uint32_t downs)
{
    for (int gpio = 0; gpio < PIN_COUNT; gpio++)
    {
        if (!(PIN_MASK & (1 << gpio)))
            continue;

        if (mask & (1 << gpio))
        {
            gpio_set_pulls(gpio, ups & (1 << gpio), downs & (1 << gpio));
        }
    }
}

#define SPI_BITBANG_MAX_TRANSFER_SIZE (1024-8)

void spi_bitbang(const spi_pins_t *pins,
                 bool handle_cs,
                 const uint32_t bit_count,
                 uint8_t const *buf_out,
                 uint8_t *buf_in)
{
    const uint32_t byte_count = (bit_count + 7)/8;

    if(byte_count > SPI_BITBANG_MAX_TRANSFER_SIZE)
        return;

    memset(buf_in, 0, SPI_BITBANG_MAX_TRANSFER_SIZE);

    if(handle_cs)
        gpio_put(pins->cs, false);
        

    for (uint32_t bit_index = 0; bit_index < bit_count; bit_index++)
    {
        const uint32_t byte_index = bit_index / 8;
        const uint8_t bit_offset = bit_index % 8;

        gpio_put(pins->mosi, (buf_out[byte_index] << bit_offset) & 0x80);
        gpio_put(pins->sck, true);

        if (gpio_get(pins->miso))
        {
            buf_in[byte_index] |= (1 << (7 - bit_offset));
        }

        gpio_put(pins->sck, false);
    }

    if(handle_cs)
        gpio_put(pins->cs, true);
}

uint32_t adc_sample_input(uint8_t input)
{
    if (input > 3)
    {
        return 0;
    }

    // Configure the GPIO
    adc_gpio_init(input + 26);

    adc_select_input(input);

    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    const uint32_t conversion_factor = 3300000 / (1 << 12); // microvolts per sample
    uint16_t result = adc_read();

    return result * conversion_factor;
}

uint32_t read_uint32(uint8_t const *buffer)
{
    const uint32_t val =
        (*(buffer + 0) << 24) + (*(buffer + 1) << 16) + (*(buffer + 2) << 8) + (*(buffer + 3) << 0);
    return val;
}

void write_uint32(uint32_t val, uint8_t *buffer)
{
    buffer[0] = ((val >> 24) & 0xFF);
    buffer[1] = ((val >> 16) & 0xFF);
    buffer[2] = ((val >> 8) & 0xFF);
    buffer[3] = ((val >> 0) & 0xFF);
}


typedef enum
{
    FLASHER_REQUEST_PIN_DIRECTION_SET = 0x10,       // Configurure GPIO pin directions
    FLASHER_REQUEST_PULLUPS_SET = 0x12,             // Configure GPIO pullups
    FLASHER_REQUEST_PIN_VALUES_SET = 0x20,          // Set GPIO output values
    FLASHER_REQUEST_PIN_VALUES_GET = 0x30,          // Get GPIO input values
    FLASHER_REQUEST_SPI_BITBANG_CS = 0x41,          // SPI transaction with CS pin
    FLASHER_REQUEST_SPI_BITBANG_NO_CS = 0x42,       // SPI transaction without CS pin
    FLASHER_REQUEST_SPI_PINS_SET = 0x43,            // Configure SPI pins
    FLASHER_REQUEST_ADC_READ = 0x50,                // Read ADC inputs
    FLASHER_REQUEST_BOOTLOADER = 0xFF               // Jump to bootloader mode
} flasher_request_t;

static const uint8_t microsoft_os_compatible_id_desc[] = {
	40, 0, 0, 0, // total length, 16 header + 24 function * 1
	0, 1, // Version 1
	4, 0, // Extended compatibility ID descriptor index
	1, // Number of function sections
	0, 0, 0, 0, 0, 0, 0, // Reserved
	0, // Interface number
	1, // Reserved
	'W','I','N','U','S','B',0,0, // compatibleID
	0,0,0,0,0,0,0,0,             // subCompatibleID
	0,0,0,0,0,0 // Reserved
};

uint8_t out_buffer[1024];
uint8_t in_buffer[1024];
uint8_t bitbang_in_buffer[SPI_BITBANG_MAX_TRANSFER_SIZE];

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    // Only handle vendor requests
    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR)
        return false;

    if ((stage == CONTROL_STAGE_SETUP) && (request->bmRequestType_bit.direction == 1))
    { // Device to host (IN)
        memset(in_buffer, 0, sizeof(in_buffer));

        switch (request->bRequest)
        {
	case 0xF8:
		// TODO: Check index
            memcpy(in_buffer,
			    microsoft_os_compatible_id_desc,
			    sizeof(microsoft_os_compatible_id_desc)
			    );
            return tud_control_xfer(rhport,
			    request,
			    (void *)(uintptr_t)in_buffer,
			    sizeof(microsoft_os_compatible_id_desc)
			    );
            break;

        case FLASHER_REQUEST_PIN_VALUES_GET:
            write_uint32(gpio_get_all() & PIN_MASK, &in_buffer[0]);
            return tud_control_xfer(rhport, request, (void *)(uintptr_t)in_buffer, 4);
            break;

        case FLASHER_REQUEST_ADC_READ:
            write_uint32(adc_sample_input(0), &in_buffer[0]);
            write_uint32(adc_sample_input(1), &in_buffer[4]);
            write_uint32(adc_sample_input(2), &in_buffer[8]);
            return tud_control_xfer(rhport, request, (void *)(uintptr_t)in_buffer, (3*4));
            break;

        case FLASHER_REQUEST_SPI_BITBANG_CS:
        case FLASHER_REQUEST_SPI_BITBANG_NO_CS:
            return tud_control_xfer(rhport, request, (void *)(uintptr_t)bitbang_in_buffer, sizeof(bitbang_in_buffer));
            break;

        default:
            break;
        }
    }
    else if ((stage == CONTROL_STAGE_SETUP) && (request->bmRequestType_bit.direction == 0))
    { // Host to device (OUT): Set up request to handle DATA stage
        switch (request->bRequest)
        {
        case FLASHER_REQUEST_PIN_DIRECTION_SET:
        case FLASHER_REQUEST_PULLUPS_SET:
        case FLASHER_REQUEST_PIN_VALUES_SET:
        case FLASHER_REQUEST_SPI_BITBANG_CS:
        case FLASHER_REQUEST_SPI_BITBANG_NO_CS:
        case FLASHER_REQUEST_SPI_PINS_SET:
            return tud_control_xfer(rhport, request, (void *)(uintptr_t)out_buffer, sizeof(out_buffer));
            break;

	case FLASHER_REQUEST_BOOTLOADER:
            reset_usb_boot(0,0);
            return true;
            break;


        default:
            break;
        }
    }
    else if ((stage == CONTROL_STAGE_ACK) && (request->bmRequestType_bit.direction == 0))
    { // Host to device (OUT): Handle data
        switch (request->bRequest)
        {
        case FLASHER_REQUEST_PIN_DIRECTION_SET:
            gpio_set_dir_masked(
                read_uint32(&out_buffer[0]) & PIN_MASK,
                read_uint32(&out_buffer[4]) & PIN_MASK);
            return true;
            break;

        case FLASHER_REQUEST_PULLUPS_SET: // set pullups/pulldowns
            set_pulls_masked(
                read_uint32(&out_buffer[0]) & PIN_MASK,
                read_uint32(&out_buffer[4]) & PIN_MASK,
                read_uint32(&out_buffer[8]) & PIN_MASK);
            return true;
            break;
            
        case FLASHER_REQUEST_PIN_VALUES_SET: // set pin values
            gpio_put_masked(
                read_uint32(&out_buffer[0]) & PIN_MASK,
                read_uint32(&out_buffer[4]) & PIN_MASK);
            return true;
            break;


        case FLASHER_REQUEST_SPI_BITBANG_CS:
            spi_bitbang(
                &spi_pins,
                true,
                read_uint32(&out_buffer[0]),
                &out_buffer[4],
                bitbang_in_buffer
                );
            
            return true;
            break;

        case FLASHER_REQUEST_SPI_BITBANG_NO_CS:
            spi_bitbang(
                &spi_pins,
                false,
                read_uint32(&out_buffer[0]),
                &out_buffer[4],
                bitbang_in_buffer
                );
            
            return true;
            break;

        case FLASHER_REQUEST_SPI_PINS_SET:
            spi_pins.sck = out_buffer[0];
            spi_pins.cs = out_buffer[1];
            spi_pins.mosi = out_buffer[2];
            spi_pins.miso = out_buffer[3];

            return true;
            break;

        default:
            break;
        }
    }
    else {
        return true;
    }

    // stall unknown request
    return false;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
    static uint32_t start_ms = 0;
    static bool led_state = false;

    // Blink every interval ms
    if (board_millis() - start_ms < blink_interval_ms)
        return; // not enough time
    start_ms += blink_interval_ms;

    board_led_write(led_state);
    led_state = 1 - led_state; // toggle
}
