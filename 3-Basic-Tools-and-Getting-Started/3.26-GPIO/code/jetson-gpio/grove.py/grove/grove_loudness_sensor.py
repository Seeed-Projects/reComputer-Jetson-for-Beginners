#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
'''
This is the code for
    - `Grove - Loudness Sensor <https://www.seeedstudio.com/Grove-Loudness-Sensor-p-1382.html>`_

Examples:

    .. code-block:: python

        import time
        from grove.grove_loudness_sensor import GroveLoudnessSensor

        # connect to alalog pin 2(slot A2)
        PIN = 2

        sensor = GroveLoudnessSensor(PIN)

        print('Detecting loud...')
        while True:
            value = sensor.value
            if value > 10:
                print("Loud value {}, Loud Detected.".format(value))
                time.sleep(.5)
'''
import time, sys, math
from grove.adc import ADC

__all__ = ["GroveLoudnessSensor"]

class GroveLoudnessSensor(object):
    '''
    Grove Loudness Sensor class

    Args:
        pin(int): number of analog pin/channel the sensor connected.
    '''
    def __init__(self, channel):
        self.channel = channel
        self.adc = ADC()

    @property
    def value(self):
        '''
        Get the loudness strength value, maximum value is 100.0%

        Returns:
            (int): ratio, 0(0.0%) - 1000(100.0%)
        '''
        return self.adc.read(self.channel)

Grove = GroveLoudnessSensor


def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.ADC)
    pin = sh.argv2pin()

    sensor = GroveLoudnessSensor(pin)

    print('Detecting loud...')
    while True:
        value = sensor.value
        if value > 10:
            print("Loud value {}, Loud Detected.".format(value))
            time.sleep(.5)

if __name__ == '__main__':
    main()
