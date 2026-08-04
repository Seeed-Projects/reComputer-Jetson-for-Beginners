#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
#
# This is the library for Grove Base Hat
# which used to connect grove sensors for Raspberry Pi.
'''
This is the code for
    - `Grove - I2C High Accuracy Temperature Sensor(MCP9808) <https://www.seeedstudio.com/Grove-I2C-High-Accuracy-Temperature-Sensor-MCP980-p-3108.html>`_

Examples:
    .. code-block:: python
    import sys
    import time
    from grove.factory import Factory
    from grove.temperature import Temper

    print("Insert Grove - I2C-High-Accuracy-Temperature")
    print("  to Grove-Base-Hat any I2C slot")

    sensor = Factory.getTemper("MCP9808-I2C")
    sensor.resolution(Temper.RES_1_16_CELSIUS)

    print('Detecting temperature...')
    while True:
        print('{} Celsius'.format(sensor.temperature))
        time.sleep(1)
'''
import sys
import time
from grove.factory import Factory
from grove.temperature import Temper

__all__ = ['Temper']

def main():
    print("Insert Grove - I2C-High-Accuracy-Temperature")
    print("  to Grove-Base-Hat any I2C slot")

    #I2C address of temperature device: 0x18 (default) to 0x1F
    i2c_id=0x18

    sensor = Factory.getTemper("MCP9808-I2C", address=i2c_id)
    sensor.resolution(Temper.RES_1_16_CELSIUS)

    print('Detecting temperature...')
    while True:
        print('{} Celsius'.format(sensor.temperature))
        time.sleep(1)


if __name__ == '__main__':
    main()

