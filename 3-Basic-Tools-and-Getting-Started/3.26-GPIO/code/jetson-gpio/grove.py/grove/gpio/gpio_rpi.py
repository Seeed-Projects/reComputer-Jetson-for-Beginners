#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
#
'''
This is the grove.gpio.GPIO implemented by RPi.GPIO.
'''
import RPi.GPIO

__all__ = ['GPIO']

RPi.GPIO.setwarnings(False)
RPi.GPIO.setmode(RPi.GPIO.BCM)


class GPIO(object):
    OUT = RPi.GPIO.OUT
    IN = RPi.GPIO.IN
    
    def __init__(self, pin, direction=None):
        self.pin = pin
        if direction is not None:
            self.dir(direction)

        self._event_handle = None

    def dir(self, direction):
        RPi.GPIO.setup(self.pin, direction)

    def write(self, output):
        RPi.GPIO.output(self.pin, output)

    def read(self):
        return RPi.GPIO.input(self.pin)

    def _on_event(self, pin):
        value = self.read()
        if self._event_handle:
            self._event_handle(pin, value)

    @property
    def on_event(self):
        return self._event_handle

    @on_event.setter
    def on_event(self, handle):
        if not callable(handle):
            return

        if self._event_handle is None:
            RPi.GPIO.add_event_detect(self.pin, RPi.GPIO.BOTH, self._on_event)

        self._event_handle = handle
