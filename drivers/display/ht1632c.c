/*
 * Copyright (c) 2021 Farit N
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT holtek_ht1632c

#include <device.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>
#include <drivers/display.h>
#include <pm/device.h>

#include "ht1632c.h"

#include <sys/printk.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(display_ht1632c, CONFIG_DISPLAY_LOG_LEVEL);

/**
 * @brief Converts nanoseconds to the number of the processor system cycles
 *
 * @param uint32_t ns Nanoseconds
 *
 */
static inline uint32_t ht1632c_ns_to_sys_clock_hw_cycles(uint32_t ns)
{
    return ((uint64_t)sys_clock_hw_cycles_per_sec() * (ns) / NSEC_PER_SEC + 1);
}

/**
 * @brief Delays the execution waiting for processor cycles
 *
 * @param dev Pointer to device data
 * @param uint32_t cycles_to_wait How many processor cycles to wait
 *
 */
static void ht1632c_delay(uint32_t cycles_to_wait)
{
    uint32_t start = k_cycle_get_32();

    // Wait until the given number of cycles have passed
    while (k_cycle_get_32() - start < cycles_to_wait) {
    }
}

/**
 * @brief Sets the CS GPIO pin
 *
 * @param dev Pointer to device data
 * @param int state 1 sets the active state, 0 - inactive (CS is active LOW)
 *
 */
static void ht1632c_set_cs_pin(struct ht1632c_data *data, int state)
{
    gpio_pin_set(data->cs_gpio, data->cs_pin, state);
}

/**
 * @brief Sets the WR GPIO pin
 *
 * @param dev Pointer to device data
 * @param int state 1 sets the active state, 0 - inactive (WR is active LOW)
 *
 */
static void ht1632c_set_wr_pin(struct ht1632c_data *data, int state)
{
    gpio_pin_set(data->wr_gpio, data->wr_pin, state);
}

/**
 * @brief Sets the DATA GPIO pin
 *
 * @param dev Pointer to device data
 * @param int state 1 sets the active state, 0 - inactive (DATA is active HIGH)
 *
 */
static void ht1632c_set_data_pin(struct ht1632c_data *data, int state)
{
    gpio_pin_set(data->data_gpio, data->data_pin, state);
}

/**
 * @brief Writes bits to HT1632C
 *
 * @param dev Pointer to device data
 * @param uint16_t bits Contains the data bits
 * @param uint`6_t bits The first bit from which to write. (If 1, only the last bit will be written. If 8, 8 bits will be written.)
 *
 */
static void ht1632c_write_bits(struct ht1632c_data *data, 
        uint16_t bits, 
        uint16_t firstbit) 
{
    bool state = false;

    while(firstbit) {
        if (bits & firstbit) {
            state = true;
        } else {
            state = false;
        }

        //set the WR pin low
        ht1632c_set_wr_pin(data, true);
        //set the next DATA bit
        ht1632c_set_data_pin(data, state);
        //wait for the half-clock cycle
        ht1632c_delay(data->delays->su);
        //the next DATA bit is read on WR going from LOW to HIGH
        ht1632c_set_wr_pin(data, false);
        //wait the next half-clock cycle
        ht1632c_delay(data->delays->clk);

        firstbit >>= 1;
    }
}

/**
 * @brief Writes a command to HT1632C
 *
 * @param dev Pointer to device data
 * @param uint8_t command Command without the first 3 bits 100
 *
 */
static void ht1632c_write_command(struct ht1632c_data *data, 
        uint8_t command)
{
    //CS down
    ht1632c_delay(data->delays->cs);
    ht1632c_set_cs_pin(data, true);
    ht1632c_delay(data->delays->su1);
    
    //100 - command mode
    ht1632c_write_bits(data, HT1632C_COMMAND_HEADER, BIT(2));
    //the command itself
    ht1632c_write_bits(data, command, BIT(7));
    //one extra bit
    ht1632c_write_bits(data, 0, BIT(0));

    //set the DATA pin low waiting for the next command
    ht1632c_set_data_pin(data, false);

    //CS UP
    ht1632c_delay(data->delays->h1);
    ht1632c_set_cs_pin(data, false);
}

/**
 * @brief Write data to display
 *
 * @param dev Pointer to device structure
 * @param x x Coordinate of the upper left corner where to write the buffer. Only 0 is supported.
 * @param y y Coordinate of the upper left corner where to write the buffer. Only 0 is supported.
 * @param desc Pointer to a structure describing the buffer layout
 * @param buf Pointer to buffer array
 *
 * @retval 0 on success else negative errno code.
 */
static int ht1632c_write(const struct device *dev,
            const uint16_t x,
            const uint16_t y,
            const struct display_buffer_descriptor *desc,
            const void *buf)
{
    struct ht1632c_data *data = (struct ht1632c_data *)dev->data;
    const uint8_t *write_buf8 = (uint8_t *)buf;
    const uint16_t *write_buf16 = (uint16_t *)buf;

    //Allowed configurations
    //32 ROW x 8 COM
    //or 24 ROW x 16 COM 

    __ASSERT((data->width == 24) || (data->width == 32), "The screen width can be 24 or 32 only");
    __ASSERT(buf != NULL, "Display buffer is not available");
    __ASSERT(x == 0, "X-coordinate has to be 0");
    __ASSERT(y == 0, "Y-coordinate has to be 0");

    //CS down
    ht1632c_delay(data->delays->cs);
    ht1632c_set_cs_pin(data, true);
    ht1632c_delay(data->delays->su1);
    
    //Writing data in the Successive Address Writing Mode
    //101-A6A5A4A3A2A1A0-D0D1D2D3-D0D1D2D3...
    //from the first RAM address to the last
    //101-0x00-D0D1D2D3-D0D1D2D3D4...

    //101 - write data mode
    ht1632c_write_bits(data, HT1632C_DATA_HEADER, BIT(2));

    //start address - 0x00
    ht1632c_write_bits(data, 0x00, BIT(6));

    for (int i = 0; i < data->width; i++) {
        if (data->width == 32) {
            ht1632c_write_bits(data, write_buf8[i], BIT(7));
        } else if (data->width == 24) {
            ht1632c_write_bits(data, write_buf16[i], BIT(15));
        }
    }

    ht1632c_set_data_pin(data, false);

    //CS UP
    ht1632c_delay(data->delays->h1);
    ht1632c_set_cs_pin(data, false);

    return 0;
}

/**
 * @brief Read data from display
 *
 * @param dev Pointer to device structure
 * @param x x Coordinate of the upper left corner where to read from
 * @param y y Coordinate of the upper left corner where to read from
 * @param desc Pointer to a structure describing the buffer layout
 * @param buf Pointer to buffer array
 *
 * @retval 0 on success else negative errno code.
 */
static int ht1632c_read(const struct device *dev, const uint16_t x,
      const uint16_t y,
      const struct display_buffer_descriptor *desc,
      void *buf)
{
    return -ENOTSUP;
}

static void *ht1632c_get_framebuffer(const struct device *dev)
{
    return NULL;
}

/**
 * Turns on the diplay LEDs
  * @param dev Pointer to device structure
 */
static int ht1632c_blanking_off(const struct device *dev)
{
    struct ht1632c_data *data = (struct ht1632c_data *)dev->data;

    ht1632c_write_command(data, HT1632_LED_ON);

    return 0;
}

/**
 * Turns off the diplay LEDs
 * @param dev Pointer to device structure
 */
static int ht1632c_blanking_on(const struct device *dev)
{
    struct ht1632c_data *data = (struct ht1632c_data *)dev->data;

    ht1632c_write_command(data, HT1632_LED_OFF);

    return 0;
}

/**
 * Set the brightness of the display in steps of 1/256, where 255 is full
 * brightness and 0 is minimal.
 *
 * @param dev Pointer to device structure
 * @param contrast Contrast in steps of 1/256
 *
 * @retval 0 on success else negative errno code.
 */
static int ht1632c_set_brightness(const struct device *dev,
                const uint8_t brightness)
{
    //The PWM level for HT1632C must be 0-15
    uint8_t pwm_level;
    struct ht1632c_data *data = (struct ht1632c_data *)dev->data;
    
    pwm_level = brightness / 16;

    if (pwm_level > 15) {
        pwm_level = 15;
    }

    ht1632c_write_command(data, HT1632_PWM + pwm_level);

    return 0;
}

static int ht1632c_set_contrast(const struct device *dev,
                const uint8_t contrast)
{
    return -ENOTSUP;
}

/**
 * @brief Get display capabilities
 *
 * @param dev Pointer to device structure
 * @param capabilities Pointer to capabilities structure to populate
 */
static void ht1632c_get_capabilities(const struct device *dev,
        struct display_capabilities *capabilities)
{
    struct ht1632c_data *data = (struct ht1632c_data *)dev->data;

    memset(capabilities, 0, sizeof(struct display_capabilities));
    capabilities->x_resolution = data->width;
    capabilities->y_resolution = data->height;

    capabilities->supported_pixel_formats = PIXEL_FORMAT_MONO01;
    capabilities->current_pixel_format = PIXEL_FORMAT_MONO01;
    capabilities->screen_info = SCREEN_INFO_X_ALIGNMENT_WIDTH;
    capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

/**
 * @brief Set pixel format used by the display
 *
 * @param dev Pointer to device structure
 * @param pixel_format Pixel format to be used by display
 *
 * @retval 0 on success else negative errno code.
 */
static int ht1632c_set_pixel_format(const struct device *dev,
        const enum display_pixel_format pixel_format)
{
    return -ENOTSUP;
}

#ifdef CONFIG_PM_DEVICE

/**
 * Turns on the chip from sleep
 * @param dev Pointer to data structure
 */
static void ht1632c_exit_sleep(struct ht1632c_data *data)
{
    ht1632c_write_command(data, HT1632_SYS_ON);
    ht1632c_write_command(data, HT1632_LED_ON);

    k_sleep(K_MSEC(120));
}

/**
 * Turns off the chip into sleep
 * @param dev Pointer to data structure
 */
static void ht1632c_enter_sleep(struct ht1632c_data *data)
{
    ht1632c_write_command(data, HT1632_SYS_DIS);
    ht1632c_write_command(data, HT1632_LED_OFF);
}

/**
 * @brief Sets a power state or returns the current device power state
 *
 * @param dev Pointer to device structure
 *
 * @retval 0 on success else negative errno code.
 */
static int ht1632c_pm_control(const struct device *dev, uint32_t ctrl_command,
        enum pm_device_state *state)
{
    int ret = 0;
    struct ht1632c_data *data = (struct ht1632c_data *)dev->data;

    switch (ctrl_command) {
    case PM_DEVICE_STATE_SET:
        if (*state == PM_DEVICE_STATE_ACTIVE) {
            ht1632c_exit_sleep(data);
            data->pm_state = PM_DEVICE_STATE_ACTIVE;
            ret = 0;
        } else {
            ht1632c_enter_sleep(data);
            data->pm_state = PM_DEVICE_STATE_LOW_POWER;
            ret = 0;
        }
        break;
    case PM_DEVICE_STATE_GET:
        *state = data->pm_state;
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}
#endif /* CONFIG_PM_DEVICE */

/**
 * @brief Inits the display driver
 *
 * @param dev Pointer to device structure
 *
 * @retval 0 on success else negative errno code.
 */
static int ht1632c_init(const struct device *dev)
{
    int err;
    uint8_t commons_command;

    printk("Configuring HT1632C");

    printk("Ticks per second %u", sys_clock_hw_cycles_per_sec());

    printk("Delay %u\n", ht1632c_ns_to_sys_clock_hw_cycles(600));

    struct ht1632c_data *data = (struct ht1632c_data *)dev->data;
    const struct ht1632c_config *config = (struct ht1632c_config *)dev->config;

    data->width = 32;
    data->height = 8;

    printk("Commons options (%u)\n", config->commons_options);

    //set delays for the 4-wire protocol
    data->delays->cs = ht1632c_ns_to_sys_clock_hw_cycles(400);
    data->delays->clk = ht1632c_ns_to_sys_clock_hw_cycles(500);
    data->delays->su = ht1632c_ns_to_sys_clock_hw_cycles(250);
    data->delays->h = ht1632c_ns_to_sys_clock_hw_cycles(250);
    data->delays->su1 = ht1632c_ns_to_sys_clock_hw_cycles(300);
    data->delays->h1 = ht1632c_ns_to_sys_clock_hw_cycles(200);

    printk("Delay CS %u\n", data->delays->cs);
    printk("Delay CLK %u\n", data->delays->clk);
    printk("Delay SU %u\n", data->delays->su);
    printk("Delay H %u\n", data->delays->h);
    printk("Delay SU1 %u\n", data->delays->su1);
    printk("Delay H1 %u\n", data->delays->h1);

    data->cs_gpio = device_get_binding(config->cs_gpio_name);
    __ASSERT(data->cs_gpio != NULL, "Failed to get CS GPIO device");

    err = gpio_pin_configure(data->cs_gpio, config->cs_pin,
        config->cs_flags | GPIO_OUTPUT_HIGH | GPIO_ACTIVE_LOW);
    __ASSERT(err == 0, "Failed to configure CS GPIO pin");

    data->wr_gpio = device_get_binding(config->wr_gpio_name);
    __ASSERT(data->wr_gpio != NULL, "Failed to get WR GPIO device");

    err = gpio_pin_configure(data->wr_gpio, config->wr_pin,
        config->wr_flags | GPIO_OUTPUT_HIGH | GPIO_ACTIVE_LOW);
    __ASSERT(err == 0, "Failed to configure WR GPIO pin");

    data->data_gpio = device_get_binding(config->data_gpio_name);
    __ASSERT(data->data_gpio != NULL, "Failed to get DATA GPIO device");

    err = gpio_pin_configure(data->data_gpio, config->data_pin,
        config->data_flags | GPIO_OUTPUT_LOW | GPIO_ACTIVE_HIGH);
    __ASSERT(err == 0, "Failed to configure DATA GPIO pin");

    data->cs_pin = config->cs_pin;
    data->wr_pin = config->wr_pin;
    data->data_pin = config->data_pin;  

    printk("Sending commands");

    ht1632c_write_command(data, HT1632_SYS_ON);
    ht1632c_write_command(data, HT1632_LED_ON);

    //HT1632C can be configured in a 32x8 or 24x16 pattern  
    //and common pad N-MOS open drain output  
    //or P-MOS open drain output  LED driver 
    switch(config->commons_options) {
        case 0x01:
            commons_command = HT1632_COM_01;
            data->width = 24;
            data->height = 16;
            break;
        case 0x10:
            commons_command = HT1632_COM_10;
            data->width = 32;
            data->height = 8;
            break;
        case 0x11:
            commons_command = HT1632_COM_11;
            data->width = 24;
            data->height = 16;
            break;
        case 0x00:
        default :
            commons_command = HT1632_COM_00;
            data->width = 32;
            data->height = 8;
    }

    ht1632c_write_command(data, commons_command);

#ifdef CONFIG_PM_DEVICE
    data->pm_state = PM_DEVICE_STATE_ACTIVE;
#endif
    
    return 0;
}


static const struct display_driver_api ht1632c_api = {
    .blanking_on = ht1632c_blanking_on,
    .blanking_off = ht1632c_blanking_off,
    .write = ht1632c_write,
    .read = ht1632c_read,
    .get_framebuffer = ht1632c_get_framebuffer,
    .set_brightness = ht1632c_set_brightness,
    .set_contrast = ht1632c_set_contrast,
    .get_capabilities = ht1632c_get_capabilities,
    .set_pixel_format = ht1632c_set_pixel_format,
};

static struct ht1632c_delays ht1632c_delays;

static struct ht1632c_data ht1632c_data = {
    .delays = &ht1632c_delays
};

static struct ht1632c_config ht1632c_config = {
    .cs_gpio_name = DT_INST_GPIO_LABEL(0, cs_gpios),
    .wr_gpio_name = DT_INST_GPIO_LABEL(0, wr_gpios),
    .data_gpio_name = DT_INST_GPIO_LABEL(0, data_gpios),
    .cs_pin = DT_INST_GPIO_PIN(0, cs_gpios),
    .wr_pin = DT_INST_GPIO_PIN(0, wr_gpios),
    .data_pin = DT_INST_GPIO_PIN(0, data_gpios),
    .cs_flags = DT_INST_GPIO_FLAGS(0, cs_gpios),
    .wr_flags = DT_INST_GPIO_FLAGS(0, wr_gpios),
    .data_flags = DT_INST_GPIO_FLAGS(0, data_gpios),
    .commons_options = DT_INST_PROP(0, commons_options)
};

DEVICE_DT_INST_DEFINE(0, &ht1632c_init,
          ht1632c_pm_control, &ht1632c_data, &ht1632c_config, APPLICATION,
          CONFIG_APPLICATION_INIT_PRIORITY, &ht1632c_api);