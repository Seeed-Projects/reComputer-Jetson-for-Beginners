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
from grove.adc import ADC

class Temper(object):
    RES_1_2_CELSIUS = 0.5
    RES_1_4_CELSIUS = 0.25
    RES_1_8_CELSIUS = 0.125
    RES_1_16_CELSIUS = 0.0625

    def __init__(self, pin):
        self._resolution = float('nan')

    def fahr2celsius(self, fahr):
        return (fahr - 32.0) / 1.8

    def celsius2fahr(self, celsius):
        return celsius * 1.8 + 32.0

    # arg & return unit in Celsius
    def resolution(self, res = None):
        if res is None:
            return self._resolution
        if self._derive_res(res):
            self._resolution = res
        return self._resolution

    # To be derived
    def _derive_res(self, res):
        return False

    # To be derived
    # return unit in Celsius
    @property
    def temperature(self):
        pass


class TemperTypedNTC(Temper):
    B = 4275.
    R0 = 100000.

    def __init__(self, channel):
        self.channel = channel
        self.adc = ADC()
        self._resolution = 1.5  # Celsius
    
    @property
    def temperature(self):
        value = self.adc.read(self.channel)
        if value <= 0 or value >= 1000:
            return float('nan')

        r = 1000. / value - 1.
        r = self.R0 * r

        return 1. / (math.log10(r / self.R0) / self.B + 1 / 298.15) - 273.15

