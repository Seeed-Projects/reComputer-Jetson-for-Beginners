#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
'''
This is the code for
    - `Grove - Temperature Sensor <https://www.seeedstudio.com/Grove-Temperature-Sensor-p-774.html>`_

Examples:

    .. code-block:: python

        import time
        from grove.factory import Factory

        # connect to alalog pin 2(slot A2)
        PIN = 2

        sensor = Factory.getTemper("NTC-ADC", PIN)

        print('Detecting temperature...')
        while True:
            print('{} Celsius'.format(sensor.temperature))
            time.sleep(1)
'''
import sys
import time
from grove.factory import Factory


def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.ADC)
    pin = sh.argv2pin()

    sensor = Factory.getTemper("NTC-ADC", pin)

    print('Detecting temperature...')
    while True:
        print('{} Celsius'.format(sensor.temperature))
        time.sleep(1)


if __name__ == '__main__':
    main()

