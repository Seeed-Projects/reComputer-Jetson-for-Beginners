#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
#
# This is the library for Grove Base Hat
# which used to connect grove sensors for Raspberry Pi.
'''
Display Base Class
'''

# sphinx autoapi required
__all__ = [
    "Display",
    "TYPE_CHAR",
    "TYPE_GRAY",
    "TYPE_COLOR",
    "MAX_GRAY"
]

TYPE_CHAR  = 0
TYPE_GRAY  = 1
TYPE_COLOR = 2

MAX_GRAY = 100

class Display(object):
    '''
    All display devices should inherit this virtual class,
    which provide infrastructure such as cursor and backlight inteface, etc.
    '''
    def __init__(self):
        self._cursor = False
        self._backlight = False

    # To be derived
    def _cursor_on(self, en):
        pass

    def cursor(self, enable = None):
        '''
        Enable or disable the backlight on display device,
        not all device support it.

        Args:
            enable (bool): Optional, ``True`` to enable, ``Flase`` to disable.
                           if not provided, only to get cursor status.

        Returns:
            bool: cursor status, ``True`` - on, ``False`` - off.
        '''
        if type(enable) == bool:
            self._cursor = enable
            self._cursor_on(enable)
        return self._cursor

    # To be derived
    def _backlight_on(self, en):
        pass

    def backlight(self, enable = None):
        '''
        Enable or disable the cursor on display device,
        not all device support it.

        Args:
            enable (bool): Optional, ``True`` to enable, ``Flase`` to disable.
                           if not provided, only to get cursor status.

        Returns:
            bool: backlight status, ``True`` - on, ``False`` - off.
        '''
        if type(enable) == bool:
            self._backlight = enable
            self._backlight_on(enable)
        return self._backlight

