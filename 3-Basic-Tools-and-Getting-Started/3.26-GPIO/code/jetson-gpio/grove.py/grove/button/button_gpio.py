#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2018  Seeed Technology Co.,Ltd. 
#
'''
 This is the code for GPIO Button event detection.
'''
from __future__ import division
import time
import threading
from grove.button import Button
from grove.gpio import GPIO

__all__ = ["ButtonTypedGpio"]

_CYCLE_PERIOD    = 0.02   # 20 ms
_SINGLE_KEY_TM   = 0.03   # 30 ms
_KEY_INTERVAL    = 0.3    # 300 ms
_LONG_KEY_TM     = 2.0    # 2s

class ButtonTypedGpio(Button):
    '''
    GPIO Button Class

    provide event checking ability to derived class,
    should not use directly by end-user.
    The checking events include:

      - Button.EV_SINGLE_CLICK
      - Button.EV_DOUBLE_CLICK
      - Button.EV_LONG_PRESS
      - Button.EV_LEVEL_CHANGED

    Args:
        pin(int)         : GPIO pin number the button connected.
        low_pressed(bool): optional, default True

            True if the the button gpio level is low when pressed.

            False if level high when pressed
    '''
    # all key states in FSM
    KEY_STATE_IDLE    = 0
    KEY_STATE_DOWN    = 1
    KEY_STATE_ONESHOT = 2
    KEY_STATE_LONG    = 3

    def __init__(self, pin, low_pressed = True):
        super(ButtonTypedGpio, self).__init__(pin)

        self.__low_press = low_pressed

        self.__state = self.KEY_STATE_IDLE
        self.__duration = 0.0
        self.__distance = _KEY_INTERVAL
        self.__thrd_exit = False
        self.__thrd = None

        self.__gpio = GPIO(pin, GPIO.IN)
        self.__gpio.on_event = self.__gpio_event

        if self.__thrd is None or not self.__thrd.is_alive():
            self.__thrd = threading.Thread( \
                    target = ButtonTypedGpio.__thrd_chk_evt, \
                    args = (self,))
            self.__thrd.setDaemon(True)
            self.__thrd.start()

    def __del__(self):
        self.__thrd_exit = True
        while self.__thrd.isAlive():
            time.sleep(_CYCLE_PERIOD / _CYCLE_UNIT)
        self.__thrd.join()

    def is_pressed(self, index = 0):
        '''
        Get the button status if it's being pressed ?

        Args:
            index(int): optional, the arg `index` not be used.

        Returns:
            (bool): True  if the button is being pressed.
                    False if not.
        '''
        v = self.__gpio.read()
        return self.__low_press != bool(v)

    # called by GPIO library
    def __gpio_event(self, pin, value):
        press = self.is_pressed()
        tm = time.time()
        self._send_event(self.EV_LEVEL_CHANGED, press, tm)

    # key event FSM(finite state machine)
    def __key_evt_fsm(self, dt):
        r = 0
        press = self.is_pressed()
        self.__distance = self.__distance + dt

        if self.__state == self.KEY_STATE_IDLE:
            if press:
                self.__duration = 0.0
                self.__state = self.KEY_STATE_DOWN
        elif self.__state == self.KEY_STATE_DOWN:
            if press:
                self.__duration = self.__duration + dt
                if self.__duration >= _SINGLE_KEY_TM:
                    self.__state = self.KEY_STATE_ONESHOT
        elif self.__state == self.KEY_STATE_ONESHOT:
            if not press:
                # print("distance {}".format(self.__distance))
                if self.__distance >= _KEY_INTERVAL:
                    r = self.EV_SINGLE_CLICK
                else:
                    r = self.EV_DOUBLE_CLICK
            else:
                self.__duration = self.__duration + dt
                # print("duration {}".format(self.__duration))
                if self.__duration >= _LONG_KEY_TM:
                    r = self.EV_LONG_PRESS
                    self.__state = self.KEY_STATE_LONG
        elif self.__state == self.KEY_STATE_LONG:
            if not press:
                self.__distance = _KEY_INTERVAL

        if not press:
            self.__state = self.KEY_STATE_IDLE

        if r == self.EV_DOUBLE_CLICK:
            self.__distance = _KEY_INTERVAL
        elif r == self.EV_SINGLE_CLICK:
            self.__distance = 0.0

        return r, press


    # Thread to check events
    def __thrd_chk_evt(self):
        '''
        # prevent dither
        time.sleep(0.01)
        v = self.__gpio.read()
        if self.__low_press == bool(v):
            return
        '''
        self.__last_time = time.time();
        while not self.__thrd_exit:
        # or self.__state != self.KEY_STATE_IDLE:
            t = time.time()
            dt, self.__last_time = t - self.__last_time, t

            r, pressed = self.__key_evt_fsm(dt)
            if r:
                self._send_event(r, pressed, t)
            time.sleep(_CYCLE_PERIOD)
