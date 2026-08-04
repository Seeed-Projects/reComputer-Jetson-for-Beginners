#!/usr/bin/env python
#
# This is the library for Grove Base Hat which used to connect grove sensors for raspberry pi.
# We use python module smbus2 instead of smbus.
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
import smbus2 as smbus
from smbus2 import i2c_msg
class Bus:
    instance = None
    MRAA_I2C = 0

    def __init__(self, bus=1):  # Use bus 1 by default, which is generally suitable for Raspberry Pi 2, 3, 4, etc.
        if not self.instance:
            self.instance = smbus.SMBus(bus)
        self.bus = bus
        self.msg = i2c_msg
        
    def __getattr__(self, name):
        return getattr(self.instance, name)

