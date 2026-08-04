#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
'''
This is the code for
    - `Grove - Slide Potentiometer <https://www.seeedstudio.com/Grove-Slide-Potentiometer-p-1196.html>`_

Examples:

    .. code-block:: python

        import time
        from grove.grove_slide_potentiometer import GroveSlidePotentiometer

        # connect to alalog pin 2(slot A2)
        PIN = 2

        sensor = GroveSlidePotentiometer(PIN)

        while True:
            print('Slide potentiometer value: {}'.format(sensor.value))
            time.sleep(.2)
'''
import math
import time
from grove.adc import ADC

__all__ = ["GroveSlidePotentiometer"]

class GroveSlidePotentiometer(ADC):
    '''
    Grove Slide Poteniometer Sensor class

    Args:
        pin(int): number of analog pin/channel the sensor connected.
    '''
    def __init__(self, channel):
        self.channel = channel
        self.adc = ADC()
    
    @property
    def value(self):
        '''
        Get the position value, max position is 100.0%.

        Returns:
            (int): ratio, 0(0.0%) - 1000(100.0%)
        '''
        return self.adc.read(self.channel)


Grove = GroveSlidePotentiometer


def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.ADC)
    pin = sh.argv2pin()

    sensor = GroveSlidePotentiometer(pin)

    while True:
        print('Slide potentiometer value: {}'.format(sensor.value))
        time.sleep(.2)


if __name__ == '__main__':
    main()
