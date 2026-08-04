#!/usr/bin/env python
#
# This library is for Grove - Buzzer(https://www.seeedstudio.com/Grove-Buzzer-p-768.html)
#
# This is the library for Grove Base Hat which used to connect grove sensors for raspberry pi.
#
# Author: Peter Yang <linsheng.yang@seeed.cc>
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
#
# Author: Sarah Knepper <sarah.knepper@intel.com>
# Copyright (c) 2015 Intel Corporation.
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
# LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
# WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
from __future__ import print_function

import time
import RPi.GPIO as GPIO

def main():
    from grove.helper import helper
    # helper.root_check()

    print("Insert Grove-Buzzer to Grove-Base-Hat slot PWM[12 13 VCC GND]")
    # Grove Base Hat for Raspberry Pi
    pin = 12
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin, GPIO.OUT)

    # create PWM instance
    pwm = GPIO.PWM(pin, 10)
    pwm.start(0) 

    chords = [1047, 1175, 1319, 1397, 1568, 1760, 1976]
    # Play sound (DO, RE, MI, etc.), pausing for 0.5 seconds between notes
    try:
        for note in chords:
            pwm.ChangeFrequency(note)
            pwm.ChangeDutyCycle(95)
            time.sleep(0.5) 
    finally:
        pwm.stop()
        GPIO.cleanup()

    print("Exiting application")

if __name__ == '__main__':
    main()