#!/usr/bin/env python
#
# Library for Grove - I2C Color Sensor V2(https://www.seeedstudio.com/Grove-I2C-Color-Sensor-V2-p-2890.html)
#
# This is the library for Grove Base Hat which used to connect grove sensors for raspberry pi.
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
import time
from grove.i2c import Bus


_CMD      = 0x80
_AUTO     = 0x20

_ENABLE   = 0x00
_ATIME    = 0x01
_WTIME    = 0x03
_AILT     = 0x04
_AIHT     = 0x06
_PERS     = 0x0C
_CONFIG   = 0x0D
_CONTROL  = 0x0F
_ID       = 0x12
_STATUS   = 0x13
_CDATA    = 0x14
_RDATA    = 0x16
_GDATA    = 0x18
_BDATA    = 0x1A

_AIEN       = 0x10
_WEN        = 0x08
_AEN        = 0x02
_PON        = 0x01

_GAINS  = (1, 4, 16, 60)


class GroveI2cColorSensorV2:
    """Driver for Grove I2C Color Sensor (TCS34725)"""

    def __init__(self, bus=1, address=0x29):
        self.address = address
        self.bus = Bus(bus)

        self.awake = False

        if self.id not in (0x44, 0x4D):
            raise ValueError('Not find a Grove I2C Color Sensor V2')

        self.set_integration_time(24)
        self.set_gain(4)

    def wakeup(self):
        enable = self._read_byte(_ENABLE)
        self._write_byte(_ENABLE, enable | _PON | _AEN)
        time.sleep(0.0024)

        self.awake = True

    def sleep(self):
        enable = self._read_byte(_ENABLE)
        self._write_byte(_ENABLE, enable & ~_PON)

        self.awake = False

    def is_awake(self):
        return self._read_byte(_ENABLE) & _PON

    def set_wait_time(self, t):
        pass

    @property
    def id(self):
        return self._read_byte(_ID)

    @property
    def integration_time(self):
        steps = 256 - self._read_byte(_ATIME)
        return steps * 2.4

    def set_integration_time(self, t):
        """Set the integration time of the sensor"""
        if t < 2.4:
            t = 2.4
        elif t > 614.4:
            t = 614.4
        
        steps = int(t / 2.4)
        self._integration_time = steps * 2.4
        self._write_byte(_ATIME, 256 - steps)

    @property
    def gain(self):
        """The gain control. Should be 1, 4, 16, or 60.
        """
        return _GAINS[self._read_byte(_CONTROL)]

    def set_gain(self, gain):
        if gain in _GAINS:
            self._write_byte(_CONTROL, _GAINS.index(gain))

    @property
    def raw(self):
        """Read RGBC registers
        return 16 bits red, green, blue and clear data
        """

        if not self.awake:
            self.wakeup()

        while not self._valid():
            time.sleep(0.0024)

        data = tuple(self._read_word(reg) for reg in (_RDATA, _GDATA, _BDATA, _CDATA))
        return data

    @property
    def rgb(self):
        """Read the RGB color detected by the sensor.  Returns a 3-tuple of
        red, green, blue component values as bytes (0-255).
        """
        r, g, b, clear = self.raw
        if clear:
            r = int(255 * r / clear)
            g = int(255 * g / clear)
            b = int(255 * b / clear)
        else:
            r, g, b = 0, 0, 0
        return r, g, b

    def _valid(self):
        """Check if RGBC is valid"""
        return self._read_byte(_STATUS) & 0x01

    def _read_byte(self, address):
        command = _CMD | address
        return self.bus.read_byte_data(self.address, command)

    def _read_word(self, address):
        command = _CMD | _AUTO | address
        return self.bus.read_word_data(self.address, command)

    def _write_byte(self, address, data):
        command = _CMD | address
        self.bus.write_byte_data(self.address, command, data)

    def _write_word(self, address, data):
        command = _CMD | _AUTO | address
        data = [(data >> 8) & 0xFF, data & 0xFF]
        self.bus.write_i2c_block_data(self.address, command, data)


Grove = GroveI2cColorSensorV2


def main():
    sensor = GroveI2cColorSensorV2()

    print('Raw data of red-filtered, green-filtered, blue-filtered and unfiltered photodiodes')
    while True:
        # r, g, b = sensor.rgb
        r, g, b, clear = sensor.raw
        print((r, g, b, clear))
        time.sleep(1.0)

if __name__ == '__main__':
    main()
