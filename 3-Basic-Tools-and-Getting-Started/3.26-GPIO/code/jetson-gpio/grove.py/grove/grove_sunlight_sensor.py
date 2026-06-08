#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2024  Seeed Technology Co.,Ltd.
'''
This is the code for
    - `Grove - sunlight Sensor <https://www.seeedstudio.com/Grove-Sunlight-Sensor.html>`_

Examples:

    .. code-block:: python

        from grove.grove_sunlight_sensor import GroveSi115xSensor
        import time

        SI1151 = GroveSi115xSensor()
        while True:
            print('Visible %03d IR %03d' % (SI1151.ReadVisible , SI1151.ReadIR),end=" ")
            print('\r', end='')
            time.sleep(0.5)

'''
from grove.i2c import Bus
from enum import IntEnum
import time

class I2C_ADDR(IntEnum):
    SI115X = 0x53

class SI115X_REG(IntEnum):
    PART_ID = 0x00
    REV_ID = 0x01
    MFR_ID = 0x02
    INFO_0 = 0x03
    INFO_1 = 0x04
    HOSTIN_3 = 0x07
    HOSTIN_2 = 0x08
    HOSTIN_0 = 0x0A
    COMMAND = 0x0B
    IRQ_ENABLE = 0x0F
    RESPONSE_0 = 0x11
    RESPONSE_1 = 0x10
    IRQ_STATUS = 0x12
    HOSTOUT_0 = 0x13
    HOSTOUT_1 = 0x14
    HOSTOUT_2 = 0x15
    HOSTOUT_3 = 0x16
    HOSTOUT_4 = 0x17
    HOSTOUT_5 = 0x18
    HOSTOUT_6 = 0x19

class SI115X_CMD(IntEnum):
    RESET_CMD_CTR = 0x00
    RESET_SW = 0x01
    FORCE = 0x11
    PAUSE = 0x12
    START = 0x13
    PARAM_GET = 0x40
    PARAM_SET = 0x80

class SI115X_PARAM(IntEnum):
    I2C_ADDR = 0x00
    CHAN_LIST = 0x01
    ADCCONFIG_0 = 0x02
    ADCSENS_0 = 0x03
    ADCPOST_0 = 0x04
    MEASCONFIG_0 = 0x05
    ADCCONFIG_1 = 0x06
    ADCPOST_1 = 0x08
    ADCSENS_1 = 0x07
    MEASCONFIG_1 = 0x09
    ADCCONFIG_2 = 0x0A
    ADCSENS_2 = 0x0B
    ADCPOST_2 = 0x0C
    MEASCONFIG_2 = 0x0D
    ADCCONFIG_3 = 0x0E
    ADCSENS_3 = 0x0F
    ADCPOST_3 = 0x10
    MEASCONFIG_3 = 0x11
    ADCCONFIG_4 = 0x12
    ADCSENS_4 = 0x13
    ADCPOST_4 = 0x14
    MEASCONFIG_4 = 0x15
    ADCCONFIG_5 = 0x16
    ADCSENS_5 = 0x17
    ADCPOST_5 = 0x18
    MEASCONFIG_5 = 0x19
    MEASRATE_H = 0x1A
    MEASRATE_L = 0x1B
    MEASCOUNT_0 = 0x1C
    MEASCOUNT_1 = 0x1D
    MEASCOUNT_2 = 0x1E
    LED1_A = 0x1F
    LED1_B = 0x20
    LED2_A = 0x21
    LED2_B = 0x22
    LED3_A = 0x23
    LED3_B = 0x24
    THRESHOLD0_H = 0x25
    THRESHOLD0_L = 0x26
    THRESHOLD1_H = 0x27
    THRESHOLD1_L = 0x28
    THRESHOLD2_H = 0x29
    THRESHOLD2_L = 0x2A
    BURST = 0x2B


class GroveSi115xSensor(object):
    def __init__(self,address = I2C_ADDR.SI115X):
        self.bus = Bus()
        self.addr = address
        assert self.Begin() , "Please check if the I2C device insert in I2C of Base Hat"
    def __del__(self):
        self.bus.close()
    def __exit__(self):
        self.bus.close()

    def Begin(self):
        if self._ReadByte(SI115X_REG.PART_ID) != 0x51:
            return False
        
        self.WriteParamData(SI115X_PARAM.CHAN_LIST, 0x5)

        # Initialize LED current
        self.WriteParamData(SI115X_PARAM.LED1_A, 0x3f)
        self.WriteParamData(SI115X_PARAM.LED1_B, 0x3f)

        # Configure ADC and enable LED drive
        self.WriteParamData(SI115X_PARAM.ADCCONFIG_0, 0x0b) # Visible
        self.WriteParamData(SI115X_PARAM.ADCSENS_0, 0x80)
        self.WriteParamData(SI115X_PARAM.ADCPOST_0, 0x0)
        self.WriteParamData(SI115X_PARAM.MEASCONFIG_0, 0)
        
        self.WriteParamData(SI115X_PARAM.ADCCONFIG_2, 0x00) # Small IR
        self.WriteParamData(SI115X_PARAM.ADCSENS_2, 0x80)
        self.WriteParamData(SI115X_PARAM.ADCPOST_2, 0x0)light
        self.WriteParamData(SI115X_PARAM.MEASCONFIG_2, 0)

        # Enable Interrupt
        self._WriteByte(SI115X_REG.IRQ_ENABLE, 0x5)
        self.SendCommand(SI115X_CMD.START)
        return True

    #read param data
    def ReadParamData(self,Reg):
        self._WriteByte(SI115X_REG.COMMAND, Reg | SI115X_CMD.PARAM_GET)
        return self._ReadByte(SI115X_REG.RESPONSE_1)

    #writ param data
    def WriteParamData(self,Reg,Value):
        while True:
            resp_pre = self._ReadByte(SI115X_REG.RESPONSE_0)
            self._WriteByte(SI115X_REG.HOSTIN_0, Value)
            self._WriteByte(SI115X_REG.COMMAND, Reg | SI115X_CMD.PARAM_SET)
            if self._ReadByte(SI115X_REG.RESPONSE_0) > resp_pre:
                break

    @property
    def ReadVisible(self):
        self.SendCommand(SI115X_CMD.FORCE)
        byte_h = self._ReadByte(SI115X_REG.HOSTOUT_0)
        byte_l = self._ReadByte(SI115X_REG.HOSTOUT_1)
        return byte_h * 256 + byte_l

    @property
    def ReadIR(self):
        self.SendCommand(SI115X_CMD.FORCE)
        byte_h = self._ReadByte(SI115X_REG.HOSTOUT_2)
        byte_l = self._ReadByte(SI115X_REG.HOSTOUT_3)
        return byte_h * 256 + byte_l

    def SendCommand(self,Command):
        while True:
            cmmnd_ctr = self._ReadByte(SI115X_REG.RESPONSE_0)
            self._WriteByte(SI115X_REG.COMMAND, Command)
            self._ReadByte(SI115X_REG.RESPONSE_0)
            r = self._ReadByte(SI115X_REG.RESPONSE_0)
            if(r > cmmnd_ctr):
                break

    # Read 8 bit data from Reg
    def _ReadByte(self,Reg):
        try:
            read_data = self.bus.read_byte_data(self.addr,Reg)
        except OSError:
            raise  OSError("Please check if the I2C device insert in I2C of Base Hat")
        return read_data

    # Write 8 bit data to Reg
    def _WriteByte(self,Reg,Value):
        try:
            self.bus.write_byte_data(self.addr,Reg,Value)
        except OSError:
            raise OSError("Please check if the I2C device insert in I2C of Base Hat")

    # read 16 bit data from Reg
    def _ReadHalfWord(self,Reg):
        try:
            block = self.bus.read_i2c_block_data(self.addr,Reg, 2)
        except OSError:
            raise OSError("Please check if the I2C device insert in I2C of Base Hat")
        read_data = (block[0] & 0xff) | (block[1] << 8)
        return read_data

def main():
    SI1151 = GroveSi115xSensor()
    while True:
        print('Visible %03d IR %03d' % (SI1151.ReadVisible, SI1151.ReadIR))
        time.sleep(0.5)

if __name__ == "__main__":
    main()
    