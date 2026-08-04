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
    - `Grove - Touch Sensor <https://www.seeedstudio.com/Grove-Touch-Sensor-p-747.html>`_

Examples:

    .. code-block:: python

        from grove.grove_touch_sensor import GroveTouchSensor
        import time

        # connect to pin 5 (slot D5)
        PIN = 5
        touch = GroveTouchSensor(PIN)

        def on_press(t):
            print('Pressed')
        def on_release(t):
            print("Released.")

        touch.on_press = on_press
        touch.on_release = on_release

        while True:
            time.sleep(1)
'''
import time
from grove.gpio import GPIO

__all__ = ['GroveTouchSensor']

class GroveTouchSensor(GPIO):
    '''
    Grove Touch Sensor class

    Args:
        pin(int): the number of gpio/slot your grove device connected.
    '''
    def __init__(self, pin):
        super(GroveTouchSensor, self).__init__(pin, GPIO.IN)
        self._last_time = time.time()

        self._on_press = None
        self._on_release = None

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
        return self._on_press

    @on_press.setter
    def on_press(self, callback):
        if not callable(callback):
            return

        if self.on_event is None:
            self.on_event = self._handle_event

        self._on_press = callback

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
        return self._on_release

    @on_release.setter
    def on_release(self, callback):
        if not callable(callback):
            return

        if self.on_event is None:
            self.on_event = self._handle_event

        self._on_release = callback

    def _handle_event(self, pin, value):
        t = time.time()
        dt, self._last_time = t - self._last_time, t

        if value:
            if callable(self._on_press):
                self._on_press(dt)
        else:
            if callable(self._on_release):
                self._on_release(dt)

Grove = GroveTouchSensor


def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.GPIO)
    pin = sh.argv2pin()

    touch = GroveTouchSensor(pin)

    def on_press(t):
        print('Pressed')
    def on_release(t):
        print("Released.")

    touch.on_press = on_press
    touch.on_release = on_release

    while True:
        time.sleep(1)


if __name__ == '__main__':
    main()
