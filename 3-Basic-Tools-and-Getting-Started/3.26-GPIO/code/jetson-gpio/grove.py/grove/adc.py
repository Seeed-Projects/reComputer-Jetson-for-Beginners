#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
# Copyright (C) 2018  Seeed Technology Co.,Ltd. 
#
# This is the ADC library for Grove Base Hat
# which used to connect grove sensors for raspberry pi.
# 
'''
This is the code for
    - `Grove Base Hat for RPi      <https://www.seeedstudio.com/Grove-WS2813-RGB-LED-Strip-Waterproof-60-LED-m-1m-p-3126.html>`_
    - `Grove Base Hat for RPi Zero <https://www.seeedstudio.com/Grove-Base-Hat-for-Raspberry-Pi-Zero-p-3187.html>`_

Grove Base Hat incorparates a micro controller STM32F030F4.

Raspberry Pi does not have ADC unit, so we use an external chip
to transmit analog data to raspberry pi.

Examples:
    .. code-block:: python

        import time
        from grove.adc import ADC

        adc = ADC()
        while True:
            # Read channel 0(Slot A0) voltage
            print(adc.read_voltage(0))
            time.sleep(1)

'''
import sys
import grove.i2c

__all__ = [
    "ADC",
    "RPI_HAT_NAME", "RPI_ZERO_HAT_NAME",
    "RPI_HAT_PID", "RPI_ZERO_HAT_PID"
]

RPI_HAT_PID      = 0x0004
RPI_ZERO_HAT_PID = 0x0005
RPI_HAT_NAME     = 'Grove Base Hat RPi'
""" The HAT name to compare with return value of :class:`ADC.name` """
RPI_ZERO_HAT_NAME= 'Grove Base Hat RPi Zero'
""" The HAT name to compare with return value of :class:`ADC.name` """


class ADC(object):
    '''
    Class ADC for the ADC unit on Grove Base Hat for RPi.

    Args:
        address(int): optional, i2c address of the ADC unit, default 0x04
    '''
    def __init__(self, address = 0x04):
        self.address = address
        self.bus = grove.i2c.Bus()

    def read_raw(self, channel):
        '''
        Read the raw data of ADC unit, with 12 bits resolution.

        Args:
            channel (int): 0 - 7, specify the channel to read

        Returns:
            (int): the adc result, in [0 - 4095]
        '''
        addr = 0x10 + channel
        return self.read_register(addr)

    # read input voltage (mV)
    def read_voltage(self, channel):
        '''
        Read the voltage data of ADC unit.

        Args:
            channel (int): 0 - 7, specify the channel to read

        Returns:
            (int): the voltage result, in mV
        '''
        addr = 0x20 + channel
        return self.read_register(addr)

    # input voltage / output voltage (%)
    def read(self, channel):
        '''
        Read the ratio between channel input voltage and power voltage (most time it's 3.3V).

        Args:
            channel (int): 0 - 7, specify the channel to read

        Returns:
            (int): the ratio, in 0.1%
        '''
        addr = 0x30 + channel
        return self.read_register(addr)

    @property
    def name(self):
        '''
        Get the Hat name.

        Returns:
            (string): could be :class:`RPI_HAT_NAME` or :class:`RPI_ZERO_HAT_NAME`
        '''
        id = self.read_register(0x0)
        if id == RPI_HAT_PID:
            return RPI_HAT_NAME
        elif id == RPI_ZERO_HAT_PID:
            return RPI_ZERO_HAT_NAME

    @property
    def version(self):
        '''
        Get the Hat firmware version.

        Returns:
            (int): firmware version
        '''
        return self.read_register(0x2)

    # read 16 bits register
    def read_register(self, n):
        '''
        Read the ADC Core (through I2C) registers

        Grove Base Hat for RPI I2C Registers

            - 0x00 ~ 0x01: 
            - 0x10 ~ 0x17: ADC raw data
            - 0x20 ~ 0x27: input voltage
            - 0x29: output voltage (Grove power supply voltage)
            - 0x30 ~ 0x37: input voltage / output voltage

        Args:
            n(int): register address.

        Returns:
            (int) : 16-bit register value.
        '''
        try:
            self.bus.write_byte(self.address, n)
            return self.bus.read_word_data(self.address, n)
        except IOError:
            print("Check whether I2C enabled and   {}  or  {}  inserted".format \
                    (RPI_HAT_NAME, RPI_ZERO_HAT_NAME))
            sys.exit(2)
            return 0


if __name__ == '__main__':
    import time

    adc = ADC()
    while True:
        print(adc.read_voltage(0))
        time.sleep(1)

