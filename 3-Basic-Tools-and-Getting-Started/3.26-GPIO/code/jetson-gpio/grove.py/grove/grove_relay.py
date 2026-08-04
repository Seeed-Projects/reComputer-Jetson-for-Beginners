#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
'''
This is the code for
    - `Grove - Relay <https://www.seeedstudio.com/s/Grove-Relay-p-769.html>`_

Examples:

    .. code-block:: python

        import time
        from grove.grove_relay import GroveRelay

        # connect to pin 5(slot D5)
        PIN   = 5
        relay = GroveRelay(PIN)

        while True:
            relay.on()
            time.sleep(1)
            relay.off()
            time.sleep(1)
'''
import time
from grove.gpio import GPIO

__all__ = ["GroveRelay"]

class GroveRelay(GPIO):
    '''
    Class for Grove - Relay

    Args:
        pin(int): number of digital pin the relay connected.
    '''
    def __init__(self, pin):
        super(GroveRelay, self).__init__(pin, GPIO.OUT)

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


Grove = GroveRelay


def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.GPIO)
    pin = sh.argv2pin()

    relay = GroveRelay(pin)

    while True:
        try:
            relay.on()
            time.sleep(1)
            relay.off()
            time.sleep(1)
        except KeyboardInterrupt:
            relay.off()
            print("exit")
            exit(1)            

if __name__ == '__main__':
    main()

