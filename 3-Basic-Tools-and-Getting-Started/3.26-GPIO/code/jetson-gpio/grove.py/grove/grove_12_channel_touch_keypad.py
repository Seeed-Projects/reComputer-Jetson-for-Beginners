#!/usr/bin/env python
#
# This is the code for Grove - 12 Key keypad Touch Sensor V2(ATTiny1616).
# which is is a multichannel proximity capacitive touch sensor.
#
'''
## License
Author: Downey
The MIT License (MIT)

Grove 12 channel touch sensor ATTiny1616 for the Raspberry Pi, used to connect grove sensors.
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
import serial


def parse_data(data):
    if(data >= 0xE1 and data <= 0xE9):
        print("The key %d is pressed" %(data-0xE0))
    elif(data == 0xEA):
        print("The key * is pressed")
    elif(data == 0xEB):
        print("The key 0 is pressed")
    elif(data == 0xEC):
        print("The key # is pressed")
    else:
        print("unexpect data")
    return

def main():
    ser = serial.Serial("/dev/ttyAMA0",9600)
    while(1):
        data = ser.read(1)
        parse_data(ord(data))


if __name__ ==  '__main__':
    main()


