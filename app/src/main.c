/*
 * Copyright (c) 2021 Farit N
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>

#include "app_version.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/display.h>

#include <sys/printk.h>

#include <pm/device.h>

#include "font_5x7.h"

//The screen width
#define APP_SCREEN_WIDTH 32

/**
 * Clears the screen buffer
 */
void clear_screen(uint8_t buf[])
{
    for (size_t i = 0; i < APP_SCREEN_WIDTH; i++) {
        buf[i] = 0x00;
    }
}

/**
 * Draws a dot "." to an indicator with an LED for a dot
 */
void inline draw_dot(uint8_t buf[], const uint8_t font[][APP_FONT_WIDTH], uint8_t c, uint8_t x)
{
    buf[x] = font[c][0];
}

/**
 * Draws a character at the position X
 */
void draw_char(uint8_t buf[], const uint8_t font[][APP_FONT_WIDTH], uint8_t c, uint8_t x) 
{
    // Convert the character to an index
    c = c & 0x7F;
    if (c < ' ') {
        c = 0;
/*
    //A special case for the dot . character
    } else if (c == '.') {
        draw_dot(buf, font, (c - ' '), x - 1);
        return;
*/
    } else {
        c -= ' ';
    }

    printk("Character index: %d\n", c);

    for (int i = x, j = 0; j < APP_FONT_WIDTH; i++, j++) {
        buf[i] = font[c][j];
        printk("i: %d, j: %d, char: %x\n", i, j, (unsigned char)font[c][j]);
    }
}

void main(void)
{
    size_t buf_size = APP_SCREEN_WIDTH;
    uint8_t buf[buf_size];

    const struct device *display_dev;
    struct display_capabilities capabilities;
    struct display_buffer_descriptor buf_desc;

    printk("Display sample for HT1632C");

    display_dev = device_get_binding(DT_LABEL(DT_INST(0, holtek_ht1632c)));

    if (display_dev == NULL) {
        printk("Device HT1632C not found. Aborting sample.");
        return;
    }

    display_get_capabilities(display_dev, &capabilities);
    printk("Display width %d", capabilities.x_resolution);
    printk("Display height %d", capabilities.y_resolution);


    buf[0] = 0b00000000;
    buf[1] = 0b11111110;
    buf[2] = 0b01000000;
    buf[3] = 0b00110000;
    buf[4] = 0b01000000;
    buf[5] = 0b11111110;

    buf_desc.buf_size = buf_size;
    buf_desc.pitch = capabilities.x_resolution;
    buf_desc.width = capabilities.x_resolution;
    buf_desc.height = capabilities.y_resolution;

    display_write(display_dev, 0, 0, &buf_desc, buf);

    for (int k = 0; k <= 255; k++) {
        display_set_brightness(display_dev, k);
        k_msleep(100);
    }

#ifdef CONFIG_PM_DEVICE
    k_msleep(10000);
    pm_device_state_set(display_dev, PM_DEVICE_STATE_OFF, NULL, NULL);
    k_msleep(10000);
    pm_device_state_set(display_dev, PM_DEVICE_STATE_ACTIVE, NULL, NULL);
#endif


    const char *str = "!\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~";

    for (size_t k = 0; k < 10; k++) {
        for (size_t i = 0; i < strlen(str); i++) {
            clear_screen(buf);
            draw_char(buf, font, str[i], 1);
            display_write(display_dev, 0, 0, &buf_desc, buf);
            k_msleep(1000);
        }
    }

    return;
}
