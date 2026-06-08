#!/usr/bin/env python
#
# This is the code for Grove - I2C Thermocouple Amplifier (MCP9600).
# (https://www.seeedstudio.com/Grove-I2C-Thermocouple-Amplifier-MCP960-p-3199.html)
#
# which is a thermocouple-to-digital converter with integrated 
# cold-junction and I2C communication protocol. 
#
# @author Downey
#         Peter Yang <turmary@126.com>
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
from __future__ import division
from grove.i2c import Bus
import math
import socket
import ctypes
import time

REG_WIDTH                      = 0x2

VERSION_ID_REG_ADDR            = 0x20
THERM_SENS_CFG_REG_ADDR        = 0x05
DEVICE_CFG_REG_ADDR            = 0x06

HOT_JUNCTION_REG_ADDR          = 0x00
JUNCTION_TEMP_DELTA_REG_ADDR   = 0X01
COLD_JUNCTION_TEMP_REG_ADDR    = 0X02

STATUS_REG_ADDR                = 0x4
_STATUS_REG_BURST_COMPLETE     = 0x80
_STATUS_REG_TEMP_UPDATED       = 0x40
_STATUS_REG_INPUT_EXCEEDED     = 0x20

FILT_OFF                       = 0
FILT_MIN                       = 1
FILT_MID                       = 4
FILT_MAX                       = 7

COLD_JUNC_RESOLUTION_0_625     = 0<<7
COLD_JUNC_RESOLUTION_0_25      = 1<<7

ADC_18BIT_RESOLUTION           = 0<<5
ADC_16BIT_RESOLUTION           = 1<<5
ADC_14BIT_RESOLUTION           = 2<<5
ADC_12BIT_RESOLUTION           = 3<<5

BURST_1_SAMPLE                 = 0<<2
BURST_2_SAMPLE                 = 1<<2
BURST_4_SAMPLE                 = 2<<2
BURST_8_SAMPLE                 = 3<<2
BURST_16_SAMPLE                = 4<<2
BURST_32_SAMPLE                = 5<<2
BURST_64_SAMPLE                = 6<<2
BURST_128_SAMPLE               = 7<<2

NORMAL_OPERATION               = 0
SHUTDOWN_MODE                  = 1
BURST_MODE                     = 2


THER_TYPE_K                    = 0X0<<4
THER_TYPE_J                    = 0X1<<4
THER_TYPE_T                    = 0X2<<4
THER_TYPE_N                    = 0X3<<4
THER_TYPE_S                    = 0X4<<4
THER_TYPE_E                    = 0X5<<4
THER_TYPE_B                    = 0X6<<4
THER_TYPE_R                    = 0X7<<4

class GroveThermocoupleAmpMCP9600(object):
    def __init__(self, address = 0x60):
        self._addr = address
        self._bus  = Bus()
        self._therm = THER_TYPE_K
        self._junc = HOT_JUNCTION_REG_ADDR
        self._junc_res = COLD_JUNC_RESOLUTION_0_625

    def _read_version(self):
        version = self._bus.read_i2c_block_data(self._addr,VERSION_ID_REG_ADDR,2)
        return version

    def _set_filt_coefficients(self,coefficients):
        data = self._bus.read_byte_data(self._addr,THERM_SENS_CFG_REG_ADDR)
        data = (data & 0xF8) | coefficients
        self._bus.write_byte_data(self._addr,THERM_SENS_CFG_REG_ADDR,data)

    def _set_cold_junc_resolution(self,junc_res):
        data = self._bus.read_byte_data(self._addr,DEVICE_CFG_REG_ADDR)
        data = (data & 0x7F) | junc_res
        self._bus.write_byte_data(self._addr,DEVICE_CFG_REG_ADDR,data)
        self._junc_res = junc_res

    def _set_ADC_meas_resolution(self,res):
        data = self._bus.read_byte_data(self._addr,DEVICE_CFG_REG_ADDR)
        data = (data & 0x9F) | res
        self._bus.write_byte_data(self._addr,DEVICE_CFG_REG_ADDR,data)

    def _set_burst_mode_samp(self,samp):
        data = self._bus.read_byte_data(self._addr,DEVICE_CFG_REG_ADDR)
        data = (data & 0xE3) | samp
        self._bus.write_byte_data(self._addr,DEVICE_CFG_REG_ADDR,data)

    def _set_sensor_mode(self,mode):
        data = self._bus.read_byte_data(self._addr,DEVICE_CFG_REG_ADDR)
        data = (data & 0xFC) | mode
        self._bus.write_byte_data(self._addr,DEVICE_CFG_REG_ADDR,data)

    def get_config(self):
        config1 = self._bus.read_byte_data(self._addr,DEVICE_CFG_REG_ADDR)
        config2 = self._bus.read_byte_data(self._addr,THERM_SENS_CFG_REG_ADDR)
        return config1, config2

    def set_therm_type(self,therm_type):
        therm_cfg_data = self._bus.read_byte_data(self._addr,THERM_SENS_CFG_REG_ADDR)
        therm_cfg_data = (therm_cfg_data & 0x8F) | therm_type
        self._bus.write_byte_data(self._addr,THERM_SENS_CFG_REG_ADDR,therm_cfg_data)
        self._therm = therm_type

    def set_junc_type(self, junc):
        if not junc is None:
            self._junc = junc
        return self._junc

    def set_config(self,
        filter     = FILT_OFF,
        junc_res   = COLD_JUNC_RESOLUTION_0_625,
        adc_res    = ADC_14BIT_RESOLUTION,
        burst_smps = BURST_1_SAMPLE,
        oper_mode  = NORMAL_OPERATION
    ):
        self._set_filt_coefficients(filter)
        self._set_cold_junc_resolution(junc_res)
        self._set_ADC_meas_resolution(adc_res)
        self._set_burst_mode_samp(burst_smps)
        self._set_sensor_mode(oper_mode)
        return None

    def read(self):
        data = self._bus.read_word_data(self._addr, self._junc)
        # Big endian -> little endian
        data = socket.ntohs(data)
        # print("RAW = 0x%X" % data)

        # It's 16-bit 2's complement code
        temperature = ctypes.c_short(data).value / 16.0
        return temperature

Grove = GroveThermocoupleAmpMCP9600

def main():
    print(
""" Make sure Grove-I2C-Thermocouple-Amplifier-(MCP9600)
   inserted in one I2C slot of Grove-Base-Hat"""
)

    snr = GroveThermocoupleAmpMCP9600()

    print("""
To prevent I2C access error,
please append below line to /boot/config.txt
--------------------------------------------
dtparam=i2c_arm_baudrate=50000
--------------------------------------------
to set a 50Kbps I2C baudrate.
""" )
    snr.set_therm_type(THER_TYPE_K)
    snr.set_config()

    while 1:
        print("Temperature: {:.2f} C".format(snr.read()))
        time.sleep(1)

if __name__  == "__main__":
    main()
