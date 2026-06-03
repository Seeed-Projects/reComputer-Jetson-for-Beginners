#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
'''
This is the code for
    - `Grove - Thumb Joystick <https://www.seeedstudio.com/Grove-Thumb-Joystick-p-935.html>`_

Examples:

    .. code-block:: python

        import time
        from grove.grove_thumb_joystick import GroveThumbJoystick

        # connect to alalog pin 2(slot A2)
        PIN = 2

        sensor = GroveThumbJoystick(PIN)

        while True:
            x, y = sensor.value
            if x > 900:
                print('Joystick Pressed')
            print("X, Y = {0} {1}".format(x, y))
            time.sleep(.2)
'''
import math
import sys
import time
from grove.adc import ADC

__all__ = ['GroveThumbJoystick']

class GroveThumbJoystick(object):
    '''
    Grove Thumb Joystick class

    Args:
        channel(int): number of analog pin/channel the sensor connected.
    '''
    def __init__(self, channel):
        self.channelX = channel
        self.channelY = channel + 1
        self.adc = ADC()

    @property
    def value(self):
        '''
        Get the water strength value

        Returns:
            (pair): x-ratio, y-ratio, all are 0(0.0%) - 1000(100.0%)
        '''
        return self.adc.read(self.channelX), self.adc.read(self.channelY)

Grove = GroveThumbJoystick


def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.ADC)
    pin = sh.argv2pin()

    sensor = GroveThumbJoystick(pin)

    while True:
        x, y = sensor.value
        if x > 900:
            print('Joystick Pressed')
        print("X, Y = {0} {1}".format(x, y))
        time.sleep(.2)

if __name__ == '__main__':
    main()
