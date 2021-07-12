/*
 * Copyright (c) 2021 Farit N
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef HT1632C_DISPLAY_DRIVER_H__
#define HT1632C_DISPLAY_DRIVER_H__

#include <zephyr.h>

// Command header 100
#define HT1632C_COMMAND_HEADER 0x04
// Data header 101
#define HT1632C_DATA_HEADER 0x05
// CMD= 0000-0000-x Turn off both system oscillator and LED duty cycle generator
#define HT1632_SYS_DIS 0x00  
//CMD= 0000-0001-x Turn on the system oscillator
#define HT1632_SYS_ON  0x01 
// CMD= 0000-0010-x Turn off the LED duty cycle generator
#define HT1632_LED_OFF 0x02
// CMD= 0000-0011-x Turn on the LED duty cycle generator
#define HT1632_LED_ON  0x03
// CMD= 0000-1000-x Turn off the blinking function
#define HT1632_BLINK_OFF  0x08
// CMD= 0000-1001-x Turn on the blinking function
#define HT1632_BLINK_ON   0x09
// CMD= 0001-00xx-x Set the slave mode and the clock source from an external clock, 
// the system clock input from the OSC pin and the synchronous signal input from the SYN pin
#define HT1632_SLAVE_MODE  0x10  
// CMD= 0001-10xx-x Set the master mode and the clock source from the on-chip RC oscillator, 
// the system clock output to OSC pin and synchronous signal output to the SYN pin
#define HT1632_RC_MASTER_MODE  0x18
// CMD= 0001-11xx-x Set the master mode and the clock source from an external clock,
// the system clock input from the OSC pin and the synchronous signal output to the SYN pin
#define HT1632_EXT_CLK_MASTER_MODE 0x1C
// CMD= 0010-ABxx-x ab=00: N-MOS open drain output and 8 COM option
#define HT1632_COM_00 0x20
// CMD= 0010-ABxx-x ab=01: N-MOS open drain output and 16 COM option
#define HT1632_COM_01 0x24
// CMD= 0010-ABxx-x ab=10: P-MOS open drain output and 8 COM option
#define HT1632_COM_10 0x28
// CMD= 0010-ABxx-x ab=11: P-MOS open drain output and 16 COM option
#define HT1632_COM_11 0x2C
// CMD= 101x-PPPP-x PWM duty cycle
// Add from 0 to 15 to increase the brightness
#define HT1632_PWM    0xA0


/** @brief Delays for the 3-wire protocol */
struct ht1632c_delays {
    //Serial interface reset pulse width
    uint32_t cs;
    //WR, RD input pulse width
    uint32_t clk;
    //Setup time for DATA to WR, RD, Clock width
    uint32_t su;
    //Hold Time for DATA to WR, RD, Clock width
    uint32_t h;
    //Setup Time for CS to WR, RD, Clock Width
    uint32_t su1;
    //Hold Time for CS to WR, RD, Clock Width
    uint32_t h1;
};

/** @brief Driver config data */
struct ht1632c_config {
    const char *cs_gpio_name;
    const char *wr_gpio_name;
    const char *data_gpio_name;
    gpio_pin_t cs_pin;
    gpio_pin_t wr_pin;
    gpio_pin_t data_pin;
    gpio_dt_flags_t cs_flags;
    gpio_dt_flags_t wr_flags;
    gpio_dt_flags_t data_flags;
    uint16_t commons_options;
};

/** @brief Driver instance data */
struct ht1632c_data {
    // The screen width in pixels
    uint16_t width;
    // The screen height in pixels
    uint16_t height;
    // GPIO used for the CS line
    const struct device *cs_gpio;
    // GPIO used for the WR line
    const struct device *wr_gpio;
    // GPIO used for the DATA line
    const struct device *data_gpio;
    // Pin on gpio used for the CS line
    gpio_pin_t cs_pin;
    // Pin on gpio used for the WR line
    gpio_pin_t wr_pin;
    // Pin on gpio used for the DATA line
    gpio_pin_t data_pin;
    // Delays 
    struct ht1632c_delays *delays;
#ifdef CONFIG_PM_DEVICE
    uint32_t pm_state;
#endif
};


#endif
