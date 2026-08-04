#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2018  Seeed Technology Co.,Ltd. 
'''
This code is for
    - `Grove - Mech Keycap <https://www.seeedstudio.com/Grove-Mech-Keycap-p-3138.html>`_

which is a mechanical switch with a build-in LED.
The 255 full color RGB LED makes it simple and easy to show the statues of your switch. 

Examples:
    .. code-block:: python

        from grove.button import Button
        import grove.grove_mech_keycap.GroveKeycap
        import time

        # slot/gpio number your device plugin
        pin = 12
        obj = GroveKeycap(pin)

        # the default behavior of led is
        #   single click - on
        #   double click - blink
        #   long press   - off
        # remove \'\'\' pairs below to begin your experiment
        \'\'\'
        # define a customized event handle your self
        def cust_on_event(index, event, tm):
            # obj.led could be used to operate led
            print("event with code {}, time {}".format(event, tm))

        obj.on_event = cust_on_event
        \'\'\'
        while True:
            time.sleep(1)
'''
import time
from grove.button import Button
from grove.factory import Factory

# sphinx autoapi required
__all__ = ["GroveKeycap"]

class GroveKeycap(object):
    '''
    Grove Mech Keycap class

    Args:
        pin(int): the number of gpio/slot your grove device connected.
                  for RPi, only 12 or 18 are acceptable.
    '''
    def __init__(self, pin):
        # High = pressed
        self.__btn = Factory.getButton("GPIO-HIGH", pin)
        # single WS2812 LED
        self.led = Factory.getOneLed("WS2812-PWM", pin + 1)
        self.__on_event = None
        self.__btn.on_event(self, GroveKeycap.__handle_event)

    @property
    def on_event(self):
        '''
        Property access with
            callback -- a callable function/object,
                        will be called when there is button event
            callback prototype:
                callback(index, code, time)
            callback argument:
                Args:
                    index(int): button index, be in 0 to [button count - 1]

                    code (int): bits combination of
                              -  Button.EV_LEVEL_CHANGED
                              -  Button.EV_SINGLE_CLICK
                              -  Button.EV_DOUBLE_CLICK
                              -  Button.EV_LONG_PRESS

                    time(time): event generation time

                Returns: none

        Examples:
            set

            .. code-block:: python

                obj.on_event = callback

            get

            .. code-block:: python

                callobj = obj.on_event
        '''
        return self.__on_event

    @on_event.setter
    def on_event(self, callback):
        if not callable(callback):
            return
        self.__on_event = callback

    def __handle_event(self, evt):
        # print("event index:{} event:{} pressed:{}"
        #      .format(evt['index'], evt['code'], evt['pressed']))
        if callable(self.__on_event):
            self.__on_event(evt['index'], evt['code'], evt['time'])
            return

        self.led.brightness = self.led.MAX_BRIGHT
        event = evt['code']
        if event & Button.EV_SINGLE_CLICK:
            self.led.light(True)
            print("turn on  LED")
        elif event & Button.EV_DOUBLE_CLICK:
            self.led.blink()
            print("blink    LED")
        elif event & Button.EV_LONG_PRESS:
            self.led.light(False)
            print("turn off LED")


Grove = GroveKeycap

def main():
    from grove import helper
    from grove.helper import helper
    helper.root_check()

    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.PWM)
    pin = sh.argv2pin()

    ledbtn = GroveKeycap(pin)

    # the default behavior of led is
    #   single click - on
    #   double click - blink
    #   long press   - off
    # remove ''' pairs below to begin your experiment
    '''
    # define a customized event handle your self
    def cust_on_event(index, event, tm):
        # obj.led could be used to operate led
        print("event with code {}, time {}".format(event, tm))

    ledbtn.on_event = cust_on_event
    '''
    while True:
        time.sleep(1)


if __name__ == '__main__':
    main()
