#!/usr/bin/env python
#
# Library for Grove - 3 Axis Accelerometer ADXL372
#

'''
## License

The MIT License (MIT)

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


from __future__ import print_function
import time
from grove.i2c import Bus


# Register Address
ADXL372_ADI_DEVID               = 0x00   #  Analog Devices, Inc., accelerometer ID 
ADXL372_MST_DEVID               = 0x01   #  Analog Devices MEMS device ID 
ADXL372_DEVID                   = 0x02   #  Device ID 
ADXL372_REVID                   = 0x03   #  product revision ID
ADXL372_STATUS_1                = 0x04   #  Status register 1 
ADXL372_STATUS_2                = 0x05   #  Status register 2 
ADXL372_FIFO_ENTRIES_2          = 0x06   #  Valid data samples in the FIFO 
ADXL372_FIFO_ENTRIES_1          = 0x07   #  Valid data samples in the FIFO 
ADXL372_X_DATA_H                = 0x08   #  X-axis acceleration data [11:4] 
ADXL372_X_DATA_L                = 0x09   #  X-axis acceleration data [3:0] | dummy LSBs 
ADXL372_Y_DATA_H                = 0x0A   #  Y-axis acceleration data [11:4] 
ADXL372_Y_DATA_L                = 0x0B   #  Y-axis acceleration data [3:0] | dummy LSBs 
ADXL372_Z_DATA_H                = 0x0C   #  Z-axis acceleration data [11:4] 
ADXL372_Z_DATA_L                = 0x0D   #  Z-axis acceleration data [3:0] | dummy LSBs 
ADXL372_X_MAXPEAK_H             = 0x15   #  X-axis MaxPeak acceleration data [15:8] 
ADXL372_X_MAXPEAK_L             = 0x16   #  X-axis MaxPeak acceleration data [7:0] 
ADXL372_Y_MAXPEAK_H             = 0x17   #  X-axis MaxPeak acceleration data [15:8] 
ADXL372_Y_MAXPEAK_L             = 0x18   #  X-axis MaxPeak acceleration data [7:0] 
ADXL372_Z_MAXPEAK_H             = 0x19   #  X-axis MaxPeak acceleration data [15:8] 
ADXL372_Z_MAXPEAK_L             = 0x1A   #  X-axis MaxPeak acceleration data [7:0] 
ADXL372_OFFSET_X                = 0x20   #  X axis offset 
ADXL372_OFFSET_Y                = 0x21   #  Y axis offset 
ADXL372_OFFSET_Z                = 0x22   #  Z axis offset 
ADXL372_X_THRESH_ACT_H          = 0x23   #  X axis Activity Threshold [15:8] 
ADXL372_X_THRESH_ACT_L          = 0x24   #  X axis Activity Threshold [7:0] 
ADXL372_Y_THRESH_ACT_H          = 0x25   #  Y axis Activity Threshold [15:8] 
ADXL372_Y_THRESH_ACT_L          = 0x26   #  Y axis Activity Threshold [7:0] 
ADXL372_Z_THRESH_ACT_H          = 0x27   #  Z axis Activity Threshold [15:8] 
ADXL372_Z_THRESH_ACT_L          = 0x28   #  Z axis Activity Threshold [7:0] 
ADXL372_TIME_ACT                = 0x29   #  Activity Time 
ADXL372_X_THRESH_INACT_H        = 0x2A   #  X axis Inactivity Threshold [15:8] 
ADXL372_X_THRESH_INACT_L        = 0x2B   #  X axis Inactivity Threshold [7:0] 
ADXL372_Y_THRESH_INACT_H        = 0x2C   #  Y axis Inactivity Threshold [15:8] 
ADXL372_Y_THRESH_INACT_L        = 0x2D   #  Y axis Inactivity Threshold [7:0] 
ADXL372_Z_THRESH_INACT_H        = 0x2E   #  Z axis Inactivity Threshold [15:8] 
ADXL372_Z_THRESH_INACT_L        = 0x2F   #  Z axis Inactivity Threshold [7:0] 
ADXL372_TIME_INACT_H            = 0x30   #  Inactivity Time [15:8] 
ADXL372_TIME_INACT_L            = 0x31   #  Inactivity Time [7:0] 
ADXL372_X_THRESH_ACT2_H         = 0x32   #  X axis Activity2 Threshold [15:8] 
ADXL372_X_THRESH_ACT2_L         = 0x33   #  X axis Activity2 Threshold [7:0] 
ADXL372_Y_THRESH_ACT2_H         = 0x34   #  Y axis Activity2 Threshold [15:8] 
ADXL372_Y_THRESH_ACT2_L         = 0x35   #  Y axis Activity2 Threshold [7:0] 
ADXL372_Z_THRESH_ACT2_H         = 0x36   #  Z axis Activity2 Threshold [15:8] 
ADXL372_Z_THRESH_ACT2_L         = 0x37   #  Z axis Activity2 Threshold [7:0] 
ADXL372_HPF                     = 0x38   #  High Pass Filter 
ADXL372_FIFO_SAMPLES            = 0x39   #  FIFO Samples 
ADXL372_FIFO_CTL                = 0x3A   #  FIFO Control 
ADXL372_INT1_MAP                = 0x3B   #  Interrupt 1 mapping control 
ADXL372_INT2_MAP                = 0x3C   #  Interrupt 2 mapping control 
ADXL372_TIMING                  = 0x3D   #  Timing 
ADXL372_MEASURE                 = 0x3E   #  Measure 
ADXL372_POWER_CTL               = 0x3F   #  Power control 
ADXL372_SELF_TEST               = 0x40   #  Self Test 
ADXL372_SRESET                  = 0x41   #  Reset 
ADXL372_FIFO_DATA               = 0x42   #  FIFO Data 

ADXL372_ADI_DEVID_VAL           = 0xAD   #  Analog Devices, Inc., accelerometer ID 
ADXL372_MST_DEVID_VAL           = 0x1D   #  Analog Devices MEMS device ID 
ADXL372_DEVID_VAL               = 0xFA   #  Device ID 
ADXL372_REVID_VAL               = 0x02   #  product revision ID


MEASURE_AUTOSLEEP_MASK          = 0xBF
MEASURE_BANDWIDTH_MASK          = 0xF8
MEASURE_ACTPROC_MASK            = 0xCF
TIMING_ODR_MASK                 = 0x1F
TIMING_WUR_MASK                 = 0xE3
PWRCTRL_OPMODE_MASK             = 0xFC
PWRCTRL_INSTON_THRESH_MASK      = 0xDF
PWRCTRL_INSTON_THRESH_MASK      = 0xDF
PWRCTRL_FILTER_SETTLE_MASK      = 0xEF

MEASURE_AUTOSLEEP_POS           = 6
MEASURE_ACTPROC_POS             = 4
TIMING_ODR_POS                  = 5
TIMING_WUR_POS                  = 2
INSTAON_THRESH_POS              = 5
FIFO_CRL_SAMP8_POS              = 0
FIFO_CRL_MODE_POS               = 1
FIFO_CRL_FORMAT_POS             = 3
PWRCTRL_FILTER_SETTLE_POS       = 4

DATA_READY          = 1
FIFO_READY          = 2
FIFO_FULL           = 4
FIFO_OVERRUN        = 8

# Acceleremoter configuration 
ACT_VALUE           = 30    # Activity threshold value
INACT_VALUE         = 30    # Inactivity threshold value
ACT_TIMER           = 1     # Activity timer value in multiples of 3.3ms
INACT_TIMER         = 1     # Inactivity timer value in multiples of 26ms
ADXL_INT1_PIN       = 7
ADXL_INT2_PIN       = 5

STANDBY_MODE        = 0
WAKEUP_MODE         = 1
INSTANT_ON_MODE     = 2
MEASUREMENT_MODE    = 3

FIFO_BYPASSED = 0
FIFO_STREAMED = 1
FIFO_TRIGGERED = 2
FIFO_OLDEST_SAVED = 3

FIFO_XYZ = 0
FIFO_X = 1
FIFO_Y = 2
FIFO_XY = 3
FIFO_Z = 4
FIFO_XZ = 5
FIFO_YZ = 6
FIFO_XYZ_PEAK = 7


ADXL372_SAMPLE_RATE = (400, 800, 1600, 3200, 6400)
ADXL372_BANDWIDTH = (200, 400, 800, 1600, 3200)
ADXL372_WAKEUP_TIME = (52, 104, 208, 512, 2048, 4096, 8192, 24576)
ADXL372_OP_MODE = (STANDBY_MODE, WAKEUP_MODE, INSTANT_ON_MODE, MEASUREMENT_MODE)


class ADXL372(object):
    def __init__(self, address=0x53, i2c=None):
        self.address = address
        self.bus = Bus(i2c)

        print('ID: {}'.format(self.id))
        self.reset()

        self._timing_control = 0
        self._power_control = 0
        self._measurement_control = 0
        self._sample_rate = 400
        self._bandwidth = 200
        self._mode = STANDBY_MODE


    def read(self):
        raw = self.bus.read_i2c_block_data(self.address, ADXL372_X_DATA_H, 6)
        return self.xyz(raw)

    def read_fifo(self, size):
        data = []
        while size > 32:
            data.extend(self.bus.read_i2c_block_data(self.address, ADXL372_FIFO_DATA, 32))
            size -= 32

        if size:
            data.extend(self.bus.read_i2c_block_data(self.address, ADXL372_FIFO_DATA, size))

        return data

    def reset(self):
        self.write_register(ADXL372_SRESET, 0x52)

    def timing_control(self, sample_rate=400, wakeup_ms=52):
        try:
            sample_bits = ADXL372_SAMPLE_RATE.index(sample_rate)
            wakeup_bits = ADXL372_WAKEUP_TIME.index(wakeup_ms)
        except ValueError:
            print('Supported sample rates: {}'.format(ADXL372_SAMPLE_RATE))
            print('Supported wakeup time: {}'.format(ADXL372_WAKEUP_TIME))
            raise ValueError('Invalid sample rate or wakeup time')
        self._timing_control = (sample_bits << TIMING_ODR_POS) | (wakeup_bits << TIMING_WUR_POS)
        self._sample_rate = sample_rate

        self.write_register(ADXL372_TIMING, self._timing_control)

    def power_control(self, mode=0, low_pass_filter=0, high_pass_filter=0):
        if mode not in ADXL372_OP_MODE:
            raise ValueError('Invalid operating mode')
        value = mode
        if not low_pass_filter:
            value |= 1 << 3
        if not high_pass_filter:
            value |= 1 << 2

        self._mode = mode
        self._power_control = value
        self.write_register(ADXL372_POWER_CTL, value)

    def measurement_control(self, bandwidth=200, low_noise=0, linkloop=0, autosleep=0):
        try:
            value = ADXL372_BANDWIDTH.index(bandwidth)
        except ValueError:
            print('Supported bandwidth: {}'.format(ADXL372_BANDWIDTH))
            raise ValueError('Invalid bandwidth')
        if low_noise:
            value |= 1 << 3
        if linkloop:
            value |= linkloop << 4
        if autosleep:
            value |= 1 << 6

        self._measurement_control = value
        self.write_register(ADXL372_MEASURE, value)

    def fifo_control(self, mode=FIFO_STREAMED, format=FIFO_XYZ, samples=0x80):
        self.write_register(ADXL372_FIFO_SAMPLES, samples & 0xFF)
        self.write_register(ADXL372_FIFO_CTL, ((samples >> 8) & 0x1) | (mode << 1) | (format << 3))

    @property
    def sample_rate(self):
        return self._sample_rate

    @sample_rate.setter
    def sample_rate(self, value):
        self._sample_rate = value
        try:
            simple_bits = ADXL372_SAMPLE_RATE.index(value)
        except ValueError:
            print('Supported sample rates: {}'.format(ADXL372_SAMPLE_RATE))
            raise ValueError('Invalid sample rate')
        self._timing_control = (self._timing_control & TIMING_ODR_MASK) | (simple_bits << TIMING_ODR_POS)
        self.write_register(ADXL372_TIMING, self._timing_control)

    @property
    def bandwidth(self):
        return self._bandwidth

    @bandwidth.setter
    def bandwidth(self, value):
        if value in ADXL372_BANDWIDTH:
            self._bandwidth = value
            bandwidth_bits = ADXL372_BANDWIDTH.index(value)
            self._measurement_control = (self._measurement_control & MEASURE_BANDWIDTH_MASK) | bandwidth_bits
        else:
            print('Supported bandwidth: {}'.format(ADXL372_BANDWIDTH))
            raise ValueError('Invalid bandwidth')

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, value):
        if value in ADXL372_OP_MODE:
            self._mode = value
            self._power_control = (self._power_control & PWRCTRL_OPMODE_MASK) | value
            self.write_register(ADXL372_POWER_CTL, value)
        else:
            raise ValueError('Invalid operating mode')

    @property
    def samples_in_fifo(self):
        data = self.bus.read_word_data(self.address, ADXL372_FIFO_ENTRIES_2)
        return ((data & 0x3) << 8) | ((data >> 8) & 0xFF)

    @property
    def status(self):
        return self.bus.read_word_data(self.address, ADXL372_STATUS_1)

    @property
    def id(self):
        return self.bus.read_i2c_block_data(self.address, ADXL372_ADI_DEVID, 4)

    
    def read_register(self, register):
        return self.bus.read_byte_data(self.address, register)

    def write_register(self, register, value):
        self.bus.write_byte_data(self.address, register, value)

    def update_register(self, register, mask, shift, value):
        data = self.read_register(register)
        data = (data & mask) | ((value << shift) & ~mask)
        self.write_register(register, data)

    def dump_registers(self):
        registers = self.bus.read_i2c_block_data(self.address, 0x39, 0x43 - 0x39)
        for register in registers:
            print(hex(register))

    def xyz(self, raw):
        value = [0] * 3
        for i in range(3):
            value[i] = (raw[2*i] << 4) | (raw[2*i+1] >> 4)
            if value[i] & 0xF00:
                value[i] = -((~value[i] & 0xFFF) + 1)

        return value



Grove3AxisAccelerometerADXL372 = ADXL372
Grove = ADXL372


def measurement_mode():
    acc = ADXL372()

    sample_rate = 400  # Hz, options: 400, 800, 1600, 3200, 6400
    acc.sample_rate = sample_rate
    acc.bandwidth = sample_rate / 2
    acc.mode = MEASUREMENT_MODE

    print('By default, the low pass filter and the high pass filter are enabled')

    print('unit - 100 mG')
    while True:
        if acc.status | DATA_READY:
            x, y, z = acc.read()
            print('{:4}, {:4}, {:4}'.format(x, y, z))
        time.sleep(1.)


def wakeup_mode():
    acc = ADXL372()

    wakeup_ms = 2048    #  52, 104, 208, 512, 2048, 4096, 8192, 24576
    acc.timing_control(wakeup_ms=wakeup_ms)
    acc.power_control(mode=WAKEUP_MODE, low_pass_filter=0, high_pass_filter=1)

    print('unit - 100 mG')
    while True:
        if acc.status | DATA_READY:
            x, y, z = acc.read()
            print('{:4}, {:4}, {:4}'.format(x, y, z))
        time.sleep(wakeup_ms / 1000.)

def main():
    acc = Grove3AxisAccelerometerADXL372()

    sample_rate = 400  # Hz
    acc.timing_control(sample_rate=sample_rate)
    acc.measurement_control(bandwidth=sample_rate/2, low_noise=1)
    acc.fifo_control(mode=FIFO_STREAMED, format=FIFO_XYZ, samples=0x81)
    acc.power_control(mode=MEASUREMENT_MODE, low_pass_filter=1, high_pass_filter=0)

    print('unit - 100 mG')
    while True:
        samples = acc.samples_in_fifo
        if samples > 12:
            # To ensure that data is not overwritten and stored out of order, 
            # at least one sample set must be left in the FIFO after every read 
            data = acc.read_fifo((samples // 6 - 1) * 6)
            for i in range(0, len(data), 6):
                x, y, z = acc.xyz(data[i:i+6])
                print('{:4}, {:4}, {:4}'.format(x, y, z))

        time.sleep(1. * 10 / sample_rate)

if __name__ == '__main__':
    main()