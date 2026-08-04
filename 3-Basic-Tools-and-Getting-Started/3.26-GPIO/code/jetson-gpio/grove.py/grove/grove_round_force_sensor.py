#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
'''
This is the code for
    - `Grove - Grove - Round Force Sensor FSR402 <https://www.seeedstudio.com/Grove-Round-Force-Sensor-FSR40-p-3110.html>`_

Examples:

    .. code-block:: python

        import time
        from grove.grove_round_force_sensor import GroveRoundForceSensor

        # connect to alalog pin 2(slot A2)
        PIN = 2

        sensor = GroveRoundForceSensor(PIN)

        while True:
            fsr = sensor.value
            print('FSR Value: {}'.format(fsr), end='')
            if fsr < 10:
                print(" - No pressure")
            elif fsr < 600:
                print(" - Light squeeze")
            else:
                print(" - Big squeeze")
            time.sleep(1.0)
'''
from __future__ import print_function
import math
import sys
import time
from grove.adc import ADC

__all__ = ["GroveRoundForceSensor"]

class GroveRoundForceSensor(ADC):
    '''
    Class for Grove - Round Force Sensor

    Args:
        pin(int): number of analog pin/channel the sensor connected.
    '''
    def __init__(self, channel):
        self.channel = channel
        self.adc = ADC()
    
    @property
    def value(self):
        '''
        Get the force strength value, the strengest value is 100.0%.

        Returns:
            (int): ratio, 0(0.0%) - 1000(100.0%)
        '''
        return self.adc.read(self.channel)


Grove = GroveRoundForceSensor


def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.ADC)
    pin = sh.argv2pin()

    sensor = GroveRoundForceSensor(pin)

    while True:
        fsr = sensor.value
        print('FSR Value: {}'.format(fsr), end='')
        if fsr < 10:
            print(" - No pressure")
        elif fsr < 600:
            print(" - Light squeeze")
        else:
            print(" - Big squeeze")
        time.sleep(1.0)


if __name__ == '__main__':
    main()
