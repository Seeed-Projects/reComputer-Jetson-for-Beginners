#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
#
# Author: Zion Orent <zorent@ics.com>
# Copyright (c) 2015 Intel Corporation.
#
'''
This is the code for
    - Grove - Optical Rotary Encoder(TCUT1600X01) <https://www.seeedstudio.com/Grove-Optical-Rotary-Encoder-TCUT1600X0-p-3142.html>`_

Examples:

    .. code-block:: python

        from grove.grove_button import GroveButton
        import time, sys

        # connect to pin 5 (slot D5)
        PIN = 5
        encoder = GroveOpticalRotaryEncoder(PIN)

        # Read the value every second and detect motion
        while True:
            print("\rPosition: {0}  ".format(encoder.position()), file=sys.stderr, end='')
            time.sleep(0.001)

'''
from __future__ import print_function
import time, sys, signal, atexit
from grove.gpio import GPIO

__all__ = ["GroveOpticalRotaryEncoder"]

# The UPM version rotaryencoder has bug result in segment fault.
# This pure python version could work well.
class GroveOpticalRotaryEncoder(object):
    '''
    Grove optical Rotary Encoder(TCUT1600X01) class

    Args:
        pin(int): the number of gpio/slot your grove device connected.
    '''
    def __init__(self, pin1, pin2 = None):
        pin2 = pin1 + 1 if pin2 is None else pin2
        self.__gpio  = GPIO(pin1, GPIO.IN)
        self.__gpio2 = GPIO(pin2, GPIO.IN)
        self.__gpio.on_event = self.__gpio_event
        self._pos = 0

    # called by GPIO library
    def __gpio_event(self, pin, value):
        v1 = self.__gpio.read()
        if not v1: return
        v2 = self.__gpio2.read()
        self._pos += 1 if v2 else -1

    def position(self, pos = None):
        '''
        set or get the position counter

        Args:
            pos(int):
                optinal, the position counter to be set.

                if not specified, only get the current counter

        Returns:
            (int): current position counter
        '''
        if not pos is None:
            self._pos = pos
        return self._pos

def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.GPIO)
    pin = sh.argv2pin()

    '''
    from upm.pyupm_rotaryencoder import RotaryEncoder as GroveOpticalRotaryEncoder
    from mraa import getGpioLookup

    mraa_pin1 = getGpioLookup("GPIO%02d" % (pin + 0))
    mraa_pin2 = getGpioLookup("GPIO%02d" % (pin + 1))

    # Instantiate a Grove Rotary Encoder, using signal pins mraa_pin1 & mraa_pin2
    myRotaryEncoder = GroveOpticalRotaryEncoder(mraa_pin1, mraa_pin2);
    '''
    myRotaryEncoder = GroveOpticalRotaryEncoder(pin)

    ## Exit handlers ##
    # This function stops python from printing a stacktrace when you hit control-C
    def SIGINTHandler(signum, frame):
        raise SystemExit

    # This function lets you run code on exit, including functions from myRotaryEncoder
    def exitHandler():
        print("Exiting")
        sys.exit(0)

    # Register exit handlers
    atexit.register(exitHandler)
    signal.signal(signal.SIGINT, SIGINTHandler)

    # Read the value every second and detect motion
    counter = 0
    while True:
        print("\rPosition: {0}  ".format(myRotaryEncoder.position()), file=sys.stderr, end='')
        counter += 1
        if counter >= 5000:
            print("")
            counter = 0
        time.sleep(0.001)

if __name__ == '__main__':
    main()
