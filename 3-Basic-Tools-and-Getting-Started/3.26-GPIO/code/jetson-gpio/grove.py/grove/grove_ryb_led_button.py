#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
#
# This is the library for Grove Base Hat
# which used to connect grove sensors for Raspberry Pi.
'''
This is the code for
    - `Grove - Red    LED Button <https://www.seeedstudio.com/Grove-Red-LED-Button-p-3096.html>`_
    - `Grove - Yellow LED Button <https://www.seeedstudio.com/Grove-Yellow-LED-Button-p-3101.html>`_
    - `Grove - Blue   LED Button <https://www.seeedstudio.com/Grove-Blue-LED-Button-p-3104.html>`_

Examples:
    .. code-block:: python

        from grove.button import Button
        import grove.grove_ryb_led_button.GroveLedButton
        import time

        # slot/gpio number your device plugin
        pin = 12
        obj = GroveLedButton(pin)

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
__all__ = ["GroveLedButton"]

class GroveLedButton(object):
    '''
    Grove Red/Yellow/Blue Led Button class

    all of them has a gpio button with low valid level of pressing,
    and a gpio led with high valid level for lighting.

    Args:
        pin(int): the number of gpio/slot your grove device connected.
    '''
    def __init__(self, pin):
        # High = light on
        self.led = Factory.getOneLed("GPIO-HIGH", pin)
        # Low = pressed
        self.btn = Factory.getButton("GPIO-LOW", pin + 1)
        self.__on_event = None
        self.btn.on_event(self, GroveLedButton.__handle_event)

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
        #       .format(evt['index'], evt['code'], evt['presesed']))
        if callable(self.__on_event):
            # the customized behavior
            self.__on_event(evt['index'], evt['code'], evt['time'])
            return

        # the default behavior
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




def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.GPIO)
    pin = sh.argv2pin()

    ledbtn = GroveLedButton(pin)

    # remove ''' pairs below to begin your experiment
    '''
    # define a customized event handle your self
    def cust_on_event(index, event, tm):
        print("event with code {}, time {}".format(event, tm))

    ledbtn.on_event = cust_on_event
    '''
    while True:
        time.sleep(1)


if __name__ == '__main__':
    main()
