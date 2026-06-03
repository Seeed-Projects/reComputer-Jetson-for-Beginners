#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
#
'''
This is the code for
    - `Grove - Switch(P) <https://www.seeedstudio.com/Grove-Switch-p-1252.html>`_

Examples:

    .. code-block:: python

        from grove.grove_switch import GroveSwitch
        import time

        # connect to pin 5 (slot D5)
        PIN = 5
        swicth = GroveSwitch(PIN)

        while True:
            if swicth.state:
                print("high")
            else:
                print("low")
            time.sleep(1)

'''
import time
from grove.gpio import GPIO

__all__ = ['GroveSwitch']

class GroveSwitch(GPIO):
    '''
    Grove Switch class

    Args:
        pin(int): the number of gpio/slot your grove device connected.
    '''
    def __init__(self, pin):
        super(GroveSwitch, self).__init__(pin, GPIO.IN)

    @property
    def state(self):
        '''
        Read only property to get switch status.

        Returns:
            (bool):

                - True:  switch high
                - False: switch low

        '''
        return bool(super(GroveSwitch, self).read())


Grove = GroveSwitch


def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.GPIO)
    pin = sh.argv2pin()

    switch = GroveSwitch(pin)

    while True:
        if switch.state:
            print("HIGH")
        else:
            print("LOW")
        time.sleep(1)


if __name__ == '__main__':
    main()
