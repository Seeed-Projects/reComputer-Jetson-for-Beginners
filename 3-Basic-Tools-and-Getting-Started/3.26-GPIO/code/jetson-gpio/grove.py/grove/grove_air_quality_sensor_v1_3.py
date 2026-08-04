#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
'''
This is the code for
    - `Grove -  Air Quality Sensor <https://www.seeedstudio.com/Grove-Air-quality-sensor-v1.3-p-2439.html)>`_

Examples:

    .. code-block:: python

        import time
        from grove.grove_air_quality_sensor_v1_3 import GroveAirQualitySensor

        # connect to alalog pin 2(slot A2)
        PIN = 2

        sensor = GroveAirQualitySensor(pin)

        print('Detecting ...') 
        while True:
            value = sensor.value        
            if value > 100:
                print("{}, High Pollution.".format(value))
            else:
                print("{}, Air Quality OK.".format(value))
            time.sleep(.1)
'''
import time, sys
from grove.adc import ADC

__all__ = ["GroveAirQualitySensor"]

class GroveAirQualitySensor(object):
    '''
    Grove Air Quality Sensor class

    Args:
        pin(int): number of analog pin/channel the sensor connected.
    '''
    def __init__(self, channel):
        self.channel = channel
        self.adc = ADC()

    @property
    def value(self):
        '''
        Get the air quality strength value, badest value is 100.0%.

        Returns:
            (int): ratio, 0(0.0%) - 1000(100.0%)
        '''
        return self.adc.read(self.channel)

Grove = GroveAirQualitySensor


def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.ADC)
    pin = sh.argv2pin()

    sensor = GroveAirQualitySensor(pin)

    print('Detecting ...') 
    while True:
        value = sensor.value        
        if value > 100:
            print("{}, High Pollution.".format(value))
        else:
            print("{}, Air Quality OK.".format(value))
        time.sleep(.1)

if __name__ == '__main__':
    main()
