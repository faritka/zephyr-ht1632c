# Zephyr HT1632C driver

This repository contains a display driver for the Holtek HT1632C or HT1632D LED chip for RTOS Zephyr.

The HT1632C chip has built-in display memory and all circuitry required to drive LEDs. 
It also has some advanced features such as PWM brightness control (applied to the entire LED array, not individual LEDs), 
blinking mode and low-power mode. The interface to HT1632 is what is described as “SPI-like” and requires only 4 wires – CS (chip select), 
WR (write clock), RD (read clock) and DATA (serial data). 
Unlike real SPI, the interface has separate clock lines for read and write and bi-directional data line.

The HT1632C driver implements the bit-banging 3-wire protocol using 3 GPIO lines. 
It supports only writing and only 1 HT1632C chip. 
Cascading support can be added by using additional CS lines.

## Getting Started

Before getting started, make sure you have a proper Zephyr development
environment. You can follow the official
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

### Documentation

[Writing HT1632 driver for RTOS Zephyr and Nucleo STM32](http://hobby.farit.ru/ht1632-driver-rtos-zephyr-nucleo-stm32/).
