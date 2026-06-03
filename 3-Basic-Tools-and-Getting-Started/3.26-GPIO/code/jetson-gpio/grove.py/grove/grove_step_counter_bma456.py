#!/usr/bin/env python
#
# This is the code for Grove - Step Counter(BMA456).
# (https://www.seeedstudio.com/Grove-Step-Counter-BMA45-p-3189.html)
# which is based on BMA456, an exetremely small, triaxial, 
# low-g high performance accelerations module.
#
# @author Peter Yang <turmary@126.com>
#
# Grove.py is the library for Grove Base Hat which used to
# connect grove sensors for raspberry pi.
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
from __future__ import division
from __future__ import print_function
from ctypes import *
from ctypes import util
import sys
from grove.i2c import Bus

BMA456_I2C_ADDR = 0x19

PERF_MODE_CIC_AVG    = 0
PERF_MODE_CONTINUOUS = 1

BW_OSR4_AVG1   = 0
BW_OSR2_AVG2   = 1
BW_NORMAL_AVG4 = 2
BW_CIC_AVG8    = 3
BW_RES_AVG16   = 4
BW_RES_AVG32   = 5
BW_RES_AVG64   = 6
BW_RES_AVG128  = 7

ODR_0_78_HZ,\
ODR_1_5_HZ, \
ODR_3_1_HZ, \
ODR_6_25_HZ,\
ODR_12_5_HZ,\
ODR_25_HZ,  \
ODR_50_HZ,  \
ODR_100_HZ, \
ODR_200_HZ, \
ODR_400_HZ, \
ODR_800_HZ, \
ODR_1600_HZ = 1,2,3,4,5,6,7,8,9,10,11,12


RANGE_2G, \
RANGE_4G, \
RANGE_8G, \
RANGE_16G = 0,1,2,3

PHONE_CONFIG = 0
WRIST_CONFIG = 1

__c_module = "bma456"

try:
    _ = util.find_library(__c_module)
    # print("libbma = {}".format(_))
    _bma = cdll.LoadLibrary(_)
except Exception:
    print("Error: module lib{}.so unusable, please install lib{}".
          format(__c_module, __c_module))
    sys.exit(1)

class GroveStepCounterBMA456(object):
    def __init__(self, addr = BMA456_I2C_ADDR):
        self._dev = _bma.rpi_bma456_alloc()
        dev_path = "/dev/i2c-{}".format(Bus().bus)
        _bma.rpi_bma456_init(self._dev,
                             dev_path,
                             addr,
                             RANGE_4G,
                             ODR_1600_HZ,
                             BW_NORMAL_AVG4,
                             PERF_MODE_CONTINUOUS)
        _bma.rpi_bma456_enable(self._dev,
                               WRIST_CONFIG,
                               0,
                               1)

    def __del__(self):
        _bma.rpi_bma456_free(self._dev)

    def get_counter(self):
        return _bma.rpi_bma456_get_counter(self._dev)

    def get_temperature(self):
        return _bma.rpi_bma456_get_temperature(self._dev)

    def get_accel(self):
        x, y, z = c_double(), c_double(), c_double()
        _bma.rpi_bma456_get_accel(self._dev,
                                  byref(x), byref(y), byref(z))
        return x.value, y.value, z.value

Grove = GroveStepCounterBMA456

def main():
    import time
    print(\
""" Make sure Grove-Step-Counter(BMA456)
   inserted in one I2C slot of Grove-Base-Hat
""")

    snr = GroveStepCounterBMA456()
    while True:
        steps = snr.get_counter()
        print("Steps:  {}".format(steps))
        x, y, z = snr.get_accel()
        print(" AX = %.2fmg AY = %.2fmg AZ = %.2fmg" % (x, y, z))
        time.sleep(1.0)

if __name__ == '__main__':
    main()
