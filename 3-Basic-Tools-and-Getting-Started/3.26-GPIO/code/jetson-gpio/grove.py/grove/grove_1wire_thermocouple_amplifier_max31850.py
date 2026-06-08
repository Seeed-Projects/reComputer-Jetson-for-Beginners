#!/usr/bin/env python
#
# This is the code for Grove - 1-Wire Thermocouple Amplifier (MAX31850K).
# (https://www.seeedstudio.com/Grove-1-Wire-Thermocouple-Amplifier-MAX31850-p-3159.html)
# which is a thermocouple-to-digital converters with
# 14-bit resolution and cold-junction compensation.
#
# author: Peter Yang <turmary@126.com>
#
# Grove.py is the library for Grove Base Hat which used to
# connect grove sensors for raspberry pi.
#
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
from __future__ import print_function
from __future__ import division
import time
import glob
import sys
import os
import re

class Grove1WireThermoAmpMAX31850(object):
    def __init__(self,
        path = "/sys/devices/w1_bus_master1/*/w1_slave"
    ):
        self._devs = glob.glob(path)

    def temperature(self):
        if not len(self._devs):
            return None
        for line in os.popen("cat " + self._devs[0]):
            # lines likes
            #78 01 b0 17 f0 ff ff ff 9f : crc=9f YES
            #78 01 b0 17 f0 ff ff ff 9f t=23500
            match = re.match("^(?:[0-9a-f][0-9a-f] ){9}t=([0-9]+)", line)
            if match is None: continue
            r = int(match.group(1))
            # print("result = %d" % r)
            return r / 1000.0
        return None


    get_temperature = temperature
    read = temperature


Grove = Grove1WireThermoAmpMAX31850

# configuration
pin = 5

def main():
    print(\
""" Make sure Grove-1-Wire-Thermocouple-Amplifier(MAX31850K)
   inserted in *** slot D{} ***
""".format(pin))

    from grove import helper
    helper.root_check()

    print("This program need overlay w1-gpio")
    from grove.helper import OverlayHelper
    w1_path = "/sys/devices/w1_bus_master1"
    oh = OverlayHelper(w1_path,
                       "w1-gpio",
                       "gpiopin=%d" % pin)
    print(oh)
    if not oh.is_installed():
        print("install overlay {} ...".format(oh.name))
        oh.install()

    print("Search MAX31850 devices ...")
    import os
    os.system("echo 1 > " + w1_path + "/w1_master_search")

    snr = Grove1WireThermoAmpMAX31850(w1_path + "/*/w1_slave")
    while True:
        print("Temperature: {:.2f} C".format(snr.read()))
        time.sleep(1.0)

if __name__ == '__main__':
    main()
