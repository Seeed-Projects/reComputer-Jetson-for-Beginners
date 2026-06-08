#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2024  Seeed Technology Co.,Ltd.

'''
This is the code for
    - `Grove - Digital Light Sensor <https://www.seeedstudio.com/Grove-Digital-Light-Sensor-TSL2561.html>`_

Examples:

    .. code-block:: python

        import time
        from grove.grove_digital_light_sensor import GroveDigitalLight
        
        sensor = GroveDigitalLight()
        while True:
            print('light: %d' %(sensor.read_light_chan0()))
            time.sleep(0.5)
'''

from grove.i2c import Bus
import time

TSL2561_DEFAULT_ADDRESS = 0x29

TSL2561_COMMAND_BIT = 0x80

# Register addresses
TSL2561_REGISTER_CONTROL = 0x00                         # Control/power register
TSL2561_REGISTER_TIMING = 0x01                          # Set integration time register
TSL2561_REGISTER_THRESHHOLDL_LOW = 0x02                 # Interrupt low threshold low-byte
TSL2561_REGISTER_THRESHHOLDL_HIGH = 0x03                # Interrupt low threshold high-byte
TSL2561_REGISTER_THRESHHOLDH_LOW = 0x04                 # Interrupt high threshold low-byte
TSL2561_REGISTER_THRESHHOLDH_HIGH = 0x05                # Interrupt high threshold high-byte
TSL2561_REGISTER_INTERRUPT = 0x06                       # Interrupt settings
TSL2561_REGISTER_CRC = 0x08                             # Factory use only
TSL2561_REGISTER_ID = 0x0A                              # TSL2561 identification setting
TSL2561_REGISTER_CHAN0_LOW = 0x0C                       # Light data channel 0, low byte
TSL2561_REGISTER_CHAN0_HIGH = 0x0D                      # Light data channel 0, high byte
TSL2561_REGISTER_CHAN1_LOW = 0x0E                       # Light data channel 1, low byte
TSL2561_REGISTER_CHAN1_HIGH = 0x0F                      # Light data channel 1, high byte
  
#
class GroveDigitalLight(object):
    def __init__(self,address = TSL2561_DEFAULT_ADDRESS):
        self._bus = Bus()
        self._addr = address
        assert self.Begin() , "Please check if the I2C device insert in I2C of Base Hat"
        
    def Begin(self):
        self._bus.write_byte_data(self._addr, TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, 0x03)
        self._bus.write_byte_data(self._addr, TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING, 0x02)
        return True
    
    # visible + infrared
    def read_light_chan0(self):
        byte_L = self._bus.read_byte_data(self._addr, TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN0_LOW)
        byte_H = self._bus.read_byte_data(self._addr, TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN0_HIGH)
        return byte_H * 256 + byte_L
    # infrared
    def read_light_chan1(self):
        byte_L = self._bus.read_byte_data(self._addr, TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN1_LOW)
        byte_H = self._bus.read_byte_data(self._addr, TSL2561_COMMAND_BIT | TSL2561_REGISTER_CHAN1_HIGH)
        return byte_H * 256 + byte_L
        
def main():
    TSL2561 = GroveDigitalLight()
    while True:
        print('light: %d' %(TSL2561.read_light_chan0()))
        time.sleep(0.5)

if __name__ == "__main__":
    main()