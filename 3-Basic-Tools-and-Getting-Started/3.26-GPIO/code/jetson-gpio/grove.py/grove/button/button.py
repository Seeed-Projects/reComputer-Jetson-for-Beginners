#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2018  Seeed Technology Co.,Ltd. 
#
'''
'''

__all__ = ["Button"]

class Button(object):
    '''
    Button Base Class

    provide event callback ability to derived class

    Args:
        pin(int): optional, for GPIO type button, it's the GPIO pin number.
    '''
    # event bits
    EV_RAW_STATUS    = 1 << 0
    EV_SINGLE_CLICK  = 1 << 1
    EV_DOUBLE_CLICK  = 1 << 2
    EV_LONG_PRESS    = 1 << 3
    EV_LEVEL_CHANGED = 1 << 4
    # EV_HAS           = 1 << 31

    pins = []

    def __init__(self, pin = 0):
        self.__on_obj = None
        self.__on_event = None
        self.__event = 0
        self.pins.append(pin)
        # To use with button array
        self._index = self.pins.index(pin)

    def get_on_event(self):
        '''
        Get the event receiving object and callback member function
        
        Returns:
            (obj, event_callback): a pair consist of event receiving object
                                   and callback member function
        '''
        return self.__on_obj, self.__on_event

    def on_event(self, obj, callback):
        '''
        Set the event receiving object and it'callback member function

        Args:
            obj(object)       : the object to receiving event
            callback(callable): a member function of `obj`,
                will be called when there is button event

                callback prototype:
                    callback(obj, evt_dict)

                callback argument:
                    Args:
                        obj(object)   : the object that is `obj` argument of :class:`on_event`

                        evt_dict(dict): the event dictionary include items:
                                        - index
                                        - code
                                        - pressed
                                        - time

                    Returns: none
        '''
        if not obj:
            return
        if not callable(callback):
            return
        self.__on_obj, self.__on_event = obj, callback

    def is_pressed(self, index = 0):
        '''
        Get the button status if it's being pressed ?

        Args:
            index(int): optional, the index number of which button to be checked.
                        must be specified only if it's a multiple button device.

        Returns:
            (bool): True  if the button is being pressed.
                    False if not.
        '''
        return False

    # call by derivate class
    def _send_event(self, event, pressed, tm):
        if not callable(self.__on_event):
            return

        evt = {
                'index': self._index,
                'code' : event,
                'pressed': pressed,
                'time' : tm,
        }
        self.__on_event(self.__on_obj, evt)
