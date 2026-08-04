#!/usr/bin/env python
#
# This library is for Grove - Time of Flight Distance Sensor VL53L0X
# (https://www.seeedstudio.com/Grove-Time-of-Flight-Distance-Sensor-VL53L0-p-3086.html)
# which is a high speed, high accuracy and long range distance sensor based on VL53L0X.
#
# This is the library for Grove Base Hat which used to connect grove sensors for Raspberry Pi.
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
from rpi_vl53l0x.vl53l0x import VL53L0X

_adapter = Bus()
GroveTofDistanceVL53L0X = VL53L0X(bus = _adapter.bus)

def main():
    print("Make sure Time-of-Flight-Distance-Sensor-VL53L0X inserted")
    print("  in one I2C slot of Grove-Base-Hat")

    vl53 = GroveTofDistanceVL53L0X
    vl53.begin()

    version = vl53.get_devver()
    print("VL53L0X_GetDeviceInfo:")
    print("  Device Type   : %s" % version["type"])
    print("  Device Name   : %s" % version["name"])
    print("  Device ID     : %s" % version["id"])
    print("  RevisionMajor : %d" % version["major"])
    print("  RevisionMinor : %d" % version["minor"])

    while True:
        st = vl53.wait_ready()
        if not st:
            continue
        print("Distance = {} mm".format(vl53.get_distance()))
        time.sleep(0.5)


if __name__ == '__main__':
    main()
