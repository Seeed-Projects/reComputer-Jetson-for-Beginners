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
from __future__ import print_function
from grove.i2c import Bus
from grove.button import Button
import threading
import time

__all__ = ["ButtonTypedI2c", "NAME_5_WAY_SWITCH", "NAME_6_POS_DIP_SWITCH"]

NAME_5_WAY_SWITCH     = "Grove-5-Way-Switch"

""" The Button name to compare with return value of :class:`ButtonTypedI2c.name` """

NAME_6_POS_DIP_SWITCH = "Grove-6-Pos-DIP-Switch"

""" The Button name to compare with return value of :class:`ButtonTypedI2c.name` """

_grove_5way_tactile_keys    = ("KEY A","KEY B","KEY C","KEY D","KEY E")
_grove_6pos_dip_switch_keys = ("POS 1","POS 2","POS 3","POS 4","POS 5","POS 6")

_CMD_GET_DEV_ID     = 0x00
_CMD_GET_DEV_EVENT  = 0x01
_CMD_EVENT_DET_MODE = 0x02
_CMD_BLOCK_DET_MODE = 0x03
_CMD_TEST_GET_VER   = 0xE2

VID_MULTI_SWITCH         = 0x2886
PID_5_WAY_TACTILE_SWITCH = 0x0002
PID_6_POS_DIP_SWITCH     = 0x0003

_CYCLE_PERIOD    = 0.08   # 80 ms

class ButtonTypedI2c(Button):
    '''
    I2C Button/Switch Array Class

    provide event checking ability to derived class,
    should not use directly by end-user.
    The checking events include:

      - Button.EV_SINGLE_CLICK
      - Button.EV_DOUBLE_CLICK
      - Button.EV_LONG_PRESS
      - Button.EV_LEVEL_CHANGED

    Args:
        address(int): optional, the I2C address of the connected device.
        evt_en(bool): optional, default True

            True:  provide event checking ability.

            False: used in poll environment, manually call :class:`ButtonTypedI2c.read`.
    '''
    def __init__(self, address = 0x03, evt_en = True):
        super(ButtonTypedI2c, self).__init__(0)

        self.bus = Bus()
        self._addr = address

        # Initialise the I2C button device
        self.dev_id = 0
        self._probe_uid()
        self._version = 0
        self.version()
        self._size = self.size()

        self._set_mode(True)

        self.__last_evt = None
        self.__last_evt = self.read()

        self.key_names = _grove_5way_tactile_keys
        if self._size == 6:
            self.key_names = _grove_6pos_dip_switch_keys

        self.__thrd_exit = False
        self.__thrd = None
        if not evt_en:
            return

        if self.__thrd is None or not self.__thrd.is_alive():
            self.__thrd = threading.Thread( \
                    target = ButtonTypedI2c.__thrd_chk_evt, \
                    args = (self,))
            self.__thrd.setDaemon(True)
            self.__thrd.start()

    def __del__(self):
        if not self.__thrd:
            return

        self.__thrd_exit = True
        while self.__thrd.isAlive():
            time.sleep(_CYCLE_PERIOD / _CYCLE_UNIT)
        self.__thrd.join()

    # Thread to check events
    def __thrd_chk_evt(self):
        self.__last_time = time.time();
        while not self.__thrd_exit:
        # or self.__state != self.KEY_STATE_IDLE:
            t = time.time()
            dt, self.__last_time = t - self.__last_time, t

            evt = self.read()
            if not evt[0]:
                time.sleep(_CYCLE_PERIOD)
                continue

            for i in range(0, self.size()):
                if evt[i + 1] & ~self.EV_RAW_STATUS:
                    pressed = bool(evt[i + 1] & self.EV_RAW_STATUS)
                    self._index = i
                    self._send_event(evt[i + 1], pressed, t)
            time.sleep(_CYCLE_PERIOD)

    def _probe_uid(self):
        ID_LEN = 4
        for tr in range(4):
            v = self.bus.read_i2c_block_data(self._addr, _CMD_GET_DEV_ID, ID_LEN)
            # print("GET_DEV_ID = {}".format(v))
            did = 0
            for i in range(ID_LEN):
                did = (did >> 8) | (int(v[i]) << 24)
            # print("DEV_ID = {:8X}".format(did))
            if (did >> 16) == VID_MULTI_SWITCH:
                self.dev_id = did
                return self.dev_id
            self.bus.read_byte(self._addr, True)

    def version(self):
        '''
        Get the device firmware version.

        Returns:
            (int): firmware version, the first version is 1
        '''
        VER_LEN = 10
        if not self.dev_id:
            return 0
        v = self.bus.read_i2c_block_data(self._addr, _CMD_TEST_GET_VER, VER_LEN)
        # print("GET_VER = {}".format(str(v)))
        version = v[6] - ord('0')
        version = version * 10 + (v[8] - ord('0'))
        # print("version = {}".format(version))
        self._version = version
        return self._version

    def size(self):
        '''
        Get the button count the device have.

        Returns:
            (int): button count
        '''
        if (self.dev_id >> 16) != VID_MULTI_SWITCH:
            return 0
        if (self.dev_id & 0xFFFF) == PID_5_WAY_TACTILE_SWITCH:
            return 5
        if (self.dev_id & 0xFFFF) == PID_6_POS_DIP_SWITCH:
            return 6
        return 0

    def name(self, index = None):
        '''
        Get the device name or specified button name

        Args:
            index(int): optional, the index number of button to get name.
                        if not specified, return the device name.

        Returns:
            (string): the name of the device or pecified button
        '''
        if (self.dev_id >> 16) != VID_MULTI_SWITCH:
            return "Invalid dev"
        if not index is None:
            if index < 0 or index >= self._size:
                return "Invalid index"
            return self.key_names[index]

        if (self.dev_id & 0xFFFF) == PID_5_WAY_TACTILE_SWITCH:
            return NAME_5_WAY_SWITCH
        if (self.dev_id & 0xFFFF) == PID_6_POS_DIP_SWITCH:
            return NAME_6_POS_DIP_SWITCH
        return "Invalid dev"

    def _set_mode(self, enable):
        if not self.dev_id:
            return None
        v = _CMD_BLOCK_DET_MODE
        if enable:
            v = _CMD_EVENT_DET_MODE
        self.bus.write_byte(self._addr, v)
        return True

    def read(self):
        '''
        Get the button array status

        Returns:
            (list): a list has the size button count + 1
                    item [0] indicate if there is a event (bit 0x80).
                        bit 0x80 set if one or more the switches have event.
                        bit 0x80 clear if no one has event.
                    item [ 1 + `index` ] indicate the event of button specified
                        by index, be bits combination of

                              -  Button.EV_LEVEL_CHANGED
                              -  Button.EV_SINGLE_CLICK
                              -  Button.EV_DOUBLE_CLICK
                              -  Button.EV_LONG_PRESS

        '''
        EVT_LEN = 4
        if not self.dev_id:
            return None
        size = EVT_LEN + self._size
        v = self.bus.read_i2c_block_data(self._addr, _CMD_GET_DEV_EVENT, size)

        if self._version > 1 or self.__last_evt is None:
            return v[EVT_LEN - 1:]

        # Fix: v0.1 will miss event BTN_EV_LEVEL_CHANGED
        #      if this API called frequently.
        for i in range(self._size):
            if (v[EVT_LEN + i] ^ self.__last_evt[1 + i]) & self.EV_RAW_STATUS:
                v[EVT_LEN + i] |= Button.EV_LEVEL_CHANGED
                v[EVT_LEN - 1] |= 0x80
        self.__last_evt = v[EVT_LEN - 1:]
        return v[EVT_LEN - 1:]

    def is_pressed(self, index = 0):
        '''
        Get the button status if it's being pressed ?

        :class:`ButtonTypedI2c.read` must be called before this api call
        when used with poll method object (created with evt_en = False).

        Args:
            index(int): optional, the index number of button to be checked.
                        must be specified for this device.

        Returns:
            (bool):
                True if the button is being pressed.
                False if not.
        '''
        return not bool(self.__last_evt[index + 1] & self.EV_RAW_STATUS)


def main():
    switch = ButtonTypedI2c(evt_en = False)

    print("{} v{} Inserted".format(switch.name(), switch.version()))
    print("A {} Button/Switch Device Ready".format(switch.size()))

    while True:
        evt = switch.read()
        print("EVENT = {}".format(evt))
        for i in range(0, switch.size()):
            print("{}: ".format(switch.name(i)), end='')
            print("{} ".format(switch.is_pressed(i) and "LOW " or "HIGH"), end='')
            if switch.name() == NAME_5_WAY_SWITCH:
                print("{} ".format(evt[i + 1] & switch.EV_RAW_STATUS and "RELEASED" or "PRESSED "), end='')
            elif switch.name() == NAME_6_POS_DIP_SWITCH:
                print("{} ".format(evt[i + 1] & switch.EV_RAW_STATUS and "OFF" or "ON "), end='')
            print(" ", end='')
            print("S" if evt[i + 1] & Button.EV_SINGLE_CLICK  else " ", end='')
            print("D" if evt[i + 1] & Button.EV_DOUBLE_CLICK  else " ", end='')
            print("L" if evt[i + 1] & Button.EV_LONG_PRESS    else " ", end='')
            print("C" if evt[i + 1] & Button.EV_LEVEL_CHANGED else " ", end='')
            print(" ", end='')
        print()
        time.sleep(1.0)

if __name__ == "__main__":
    main()
