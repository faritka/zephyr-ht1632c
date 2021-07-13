/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
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

void main(void)
{
    size_t buf_size = 32;
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

    return;
}
