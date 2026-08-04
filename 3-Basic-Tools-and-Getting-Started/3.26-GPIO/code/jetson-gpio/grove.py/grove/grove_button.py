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
    - `Grove - Button <https://www.seeedstudio.com/s/Grove-Button-p-766.html>`_

Examples:

    .. code-block:: python

        from grove.grove_button import GroveButton
        import time

        # connect to pin 5 (slot D5)
        PIN = 5
        button = GroveButton(PIN)

        def on_press(t):
            print('Button is pressed')
        def on_release(t):
            print("Button is released, pressed for {0} seconds".format(round(t,6)))

        button.on_press = on_press
        button.on_release = on_release

        while True:
            time.sleep(1)
'''
import time
from grove.button import Button
from grove.factory import Factory

__all__ = ['GroveButton']

class GroveButton(object):
    '''
    Grove Button class

    Args:
        pin(int): the number of gpio/slot your grove device connected.
    '''
    def __init__(self, pin):
        # High = pressed
        self.__btn = Factory.getButton("GPIO-HIGH", pin)
        self.__last_time = time.time()
        self.__on_press = None
        self.__on_release = None
        self.__btn.on_event(self, GroveButton.__handle_event)

    @property
    def on_press(self):
        '''
        Property access with
            callback -- a callable function/object,
                        will be called when there is a button pressing.
            callback prototype:
                callback()
                Returns: none

        Examples:
            set

            .. code-block:: python

                obj.on_press = callback

            get

            .. code-block:: python

                callobj = obj.on_press
        '''
        return self.__on_press

    @on_press.setter
    def on_press(self, callback):
        if not callable(callback):
            return
        self.__on_press = callback

    @property
    def on_release(self):
        '''
        Property access with
            callback -- a callable function/object,
                        will be called when there is a button releasing.
            callback prototype:
                callback()
                Returns: none

        Examples:
            set

            .. code-block:: python

                obj.on_release = callback

            get

            .. code-block:: python

                callobj = obj.on_release
        '''
        return self.__on_release

    @on_release.setter
    def on_release(self, callback):
        if not callable(callback):
            return
        self.__on_release = callback

    def __handle_event(self, evt):
        dt, self.__last_time = evt["time"] - self.__last_time, evt["time"]
        # print("event index:{} event:{} pressed:{}"
        #      .format(evt["index"], evt["code"], evt["pressed"]))
        if evt["code"] == Button.EV_LEVEL_CHANGED:
            if evt["pressed"]:
                if callable(self.__on_press):
                    self.__on_press(dt)
            else:
                if callable(self.__on_release):
                    self.__on_release(dt)


Grove = GroveButton

def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.GPIO)
    pin = sh.argv2pin()

    button = GroveButton(pin)

    def on_press(t):
        print('Button is pressed')
    def on_release(t):
        print("Button is released, pressed for {0} seconds".format(round(t,6)))

    button.on_press = on_press
    button.on_release = on_release

    while True:
        time.sleep(1)


if __name__ == '__main__':
    main()
