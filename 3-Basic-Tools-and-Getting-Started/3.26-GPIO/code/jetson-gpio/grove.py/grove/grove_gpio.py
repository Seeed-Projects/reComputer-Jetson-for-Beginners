#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
'''
'''
import time
from grove.gpio import GPIO

__all__ = ["GroveGpio"]

class GroveGpio(GPIO):
    '''
    Class for Grove - Relay

    Args:
        pin(int): number of digital pin the relay connected.
    '''
    def __init__(self, pin):
        super(GroveGpio, self).__init__(pin, GPIO.OUT)

    def on(self):
        '''
        enable/on the relay
        '''
        self.write(1)

    def off(self):
        '''
        disable/off the relay
        '''
        self.write(0)


Grove = GroveGpio


def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.GPIO)
    pin = sh.argv2pin()

    io = GroveGpio(pin)

    while True:
        try:
            io.on()
            time.sleep(1)
            io.off()
            time.sleep(1)
        except KeyboardInterrupt:
            io.off()
            print("exit")
            exit(1)            

if __name__ == '__main__':
    main()

