#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
'''
This is the code for
    - `Grove - Light Sensor <https://www.seeedstudio.com/Grove-Light-Sensor-v1.2-p-2727.html>`_

Examples:

    .. code-block:: python

        import time
        from grove.grove_light_sensor import GroveLightSensor

        # connect to alalog pin 2(slot A2)
        PIN = 2

        sensor = GroveLightSensor(pin)

        print('Detecting light...')
        while True:
            print('Light value: {0}'.format(sensor.light))
            time.sleep(1)
'''
import time, sys, math
from grove.adc import ADC

__all__ = ["GroveLightSensor"]

class GroveLightSensor(object):
    '''
    Grove Light Sensor class

    Args:
        pin(int): number of analog pin/channel the sensor connected.
    '''
    def __init__(self, channel):
        self.channel = channel
        self.adc = ADC()

    @property
    def light(self):
        '''
        Get the light strength value, maximum value is 100.0%

        Returns:
            (int): ratio, 0(0.0%) - 1000(100.0%)
        '''
        value = self.adc.read(self.channel)
        return value

Grove = GroveLightSensor


def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.ADC)
    pin = sh.argv2pin()

    sensor = GroveLightSensor(pin)

    print('Detecting light...')
    while True:
        print('Light value: {0}'.format(sensor.light))
        time.sleep(1)

if __name__ == '__main__':
    main()
