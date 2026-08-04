#!/usr/bin/env python
#
# Library for i2c 8 channels hub TCA9548A
#

'''
## License

The MIT License (MIT)

Copyright (C) 2022  Seeed Technology Co.,Ltd. 

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

#          Channel STATUS
#  -------------------------------------------------------
# ┆  B7  ┆  B6  ┆  B5  ┆  B4  ┆  B3  ┆  B2  ┆  B1  ┆  B0  ┆                                                                
# ┆-------------------------------------------------------┆
# ┆  ch7 ┆  ch6 ┆ ch5  ┆  ch4 ┆  ch3 ┆  ch2 ┆  ch1 ┆  ch0 ┆ 
# ˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉˉ  




class TCA9548A(object):
    
    __TCA_CHANNEL_0=0x1
    __TCA_CHANNEL_1=0x2
    __TCA_CHANNEL_2=0x4
    __TCA_CHANNEL_3=0x8
    __TCA_CHANNEL_4=0x10
    __TCA_CHANNEL_5=0x20
    __TCA_CHANNEL_6=0x40
    __TCA_CHANNEL_7=0x80
    __TCA_CHANNEL_ALL=0xFF


    def __init__(self, address=0x70, i2c=None):
        self.address = address
        self.bus = Bus(i2c)

        print('ID: {}'.format(self.id))

        self._channels = 0
    
    def write(self, value):
        self.bus.write_i2c_block_data(self.address, value, [])
    
    def read(self):
        self._channels = self.bus.read_i2c_block_data(self.address, 1, 1)[0]

    def open_channel(self, channel):
        self._channels |= channel
        self.write(self._channels)
    
    def close_channel(self, channel):
        self._channels &= ~channel
        self.write(self._channels)
        
Grove = TCA9548A

def main():
    Grove.open_channel(Grove.__TCA_CHANNEL_0)
    i2c = Bus()
    
    for address in range(0, 127):
        try:
            rez = i2c.write_quick(address)
            print('0x{:02x} : {}'.format(address, rez))
        except IOError:
            pass

if __name__ == '__main__':
    main()