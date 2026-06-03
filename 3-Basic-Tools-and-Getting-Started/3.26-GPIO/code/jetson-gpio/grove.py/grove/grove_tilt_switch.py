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
    - `Grove - Tilt Switch <https://www.seeedstudio.com/Grove-Tilt-Switch-p-771.html>`_

Examples:

    .. code-block:: python

        from grove.grove_tilt_switch import GroveTiltSwitch
        import time

        # connect to pin 5 (slot D5)
        PIN = 5

        swicth = GroveTiltSwitch(pin)

        def on_trigger():
            print('Triggered')
        def on_release():
            print("Released.")

        swicth.on_trigger = on_trigger
        swicth.on_release = on_release

        while True:
            time.sleep(1)
'''
import time
from grove.gpio import GPIO

__all__ = ["GroveTiltSwitch"]

class GroveTiltSwitch(GPIO):
    '''
    Grove Tilt Switch class

    Args:
        pin(int): the number of gpio/slot your grove device connected.
    '''
    def __init__(self, pin):
        super(GroveTiltSwitch, self).__init__(pin, GPIO.IN)
        self._on_trigger = None
        self._on_release = None

    @property
    def on_trigger(self):
        '''
        Property access with
            callback -- a callable function/object,
                        will be called when there is a switch shorting/on.
            callback prototype:
                callback()
                Returns: none

        Examples:
            set

            .. code-block:: python

                obj.on_trigger = callback

            get

            .. code-block:: python

                callobj = obj.on_trigger
        '''
        return self._on_trigger

    @on_trigger.setter
    def on_trigger(self, callback):
        if not callable(callback):
            return

        if self.on_event is None:
            self.on_event = self._handle_event

        self._on_trigger = callback

    @property
    def on_release(self):
        '''
        Property access with
            callback -- a callable function/object,
                        will be called when there is a switch releasing.
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

        if value:
            if callable(self._on_trigger):
                self._on_trigger()
        else:
            if callable(self._on_release):
                self._on_release()

Grove = GroveTiltSwitch


def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.GPIO)
    pin = sh.argv2pin()

    swicth = GroveTiltSwitch(pin)

    def on_trigger():
        print('Triggered')
    def on_release():
        print("Released.")

    swicth.on_trigger = on_trigger
    swicth.on_release = on_release

    while True:
        time.sleep(1)


if __name__ == '__main__':
    main()
