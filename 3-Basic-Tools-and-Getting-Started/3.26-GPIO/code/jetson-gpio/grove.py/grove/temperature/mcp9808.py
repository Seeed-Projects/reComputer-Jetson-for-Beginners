#!/usr/bin/env python
#
# This is the library for Grove Base Hat.
#
# Temper(ature) Classes
#

'''
## License

The MIT License (MIT)

Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
Copyright (C) 2018  Seeed Technology Co.,Ltd. 

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
'''
import math
import threading
from grove.i2c import Bus 
from grove.temperature import Temper

RES_LOW = 0x00
RES_MEDIUM = 0x01
RES_HIGH = 0x02
RES_PRECISION = 0x03

MCP9808_REG_AMBIENT_TEMP = 0x05

class TemperMCP9808(Temper):
    def __init__(self, address=0x18):
        self._addr = address
        self._bus = Bus()
        self._resolution = Temper.RES_1_2_CELSIUS

    def _derive_res(self, res):
        ares = -1
        if res >= Temper.RES_1_2_CELSIUS:
            ares = RES_LOW
        elif res >= Temper.RES_1_4_CELSIUS:
            ares = RES_MEDIUM
        elif res >= Temper.RES_1_8_CELSIUS:
            ares = RES_HIGH
        elif res >= Temper.RES_1_16_CELSIUS:
            ares = RES_PRECISION

        if ares < 0:
            return False
        self._bus.write_byte(self._addr, ares)
        # print("ares = {}".format(ares))
        return True

    @property
    def temperature(self):
        result = self._bus.read_word_data(self._addr, MCP9808_REG_AMBIENT_TEMP)
        # Swap the bytes
        data = (result & 0xff) << 8 | (result & 0xff00) >> 8
        # print("data = {}".format(data))
        # print("data = {}".format(hex(data)))
        # Check if the temperature is negative
        if data & 0x1000:
            data = -((data ^ 0x0FFF) + 1)
        else:
            data = data & 0x0fff
        return data / 16.0