#!/usr/bin/env python
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
from grove.gpio import GPIO

class GPIOWrapper(GPIO):
    def __init__(self, pin, wrapper):
        self.__high_en = wrapper["high-enable"]
        self.__dir     = wrapper["direction"]
        self.__status_attr = ""
        self.__enable_attr = ""
        self.__disable_attr = ""
        if self.__dir == GPIO.IN:
            self.__status_attr  = wrapper["status-attr"]
        else:
            self.__enable_attr  = wrapper["enable-attr"]
            self.__disable_attr = wrapper["disable-attr"]
        super(GPIOWrapper, self).__init__(pin, self.__dir)

    def __is_enabled(self):
        v = self.read()
        return self.__high_en == bool(v)

    def is_enabled(self):
        return self.__is_enabled()

    def __enable(self):
        self.write(1 if self.__high_en else 0)

    def __disable(self):
        self.write(0 if self.__high_en else 1)

    def enable(self, _enable):
        if _enable:
            self.__enable()
        else:
            self.__disable()
        return

    def __getattr__(self, attr):
        if attr == self.__status_attr:
            return self.__is_enabled
        elif attr == self.__enable_attr:
            return self.__enable
        elif attr == self.__disable_attr:
            return self.__disable
        else:
            raise AttributeError(attr)

    def __setattr__(self, attr, value):
        if attr == "enable":
            self.enable(value)
        else:
            self.__dict__[attr] = value

def main():
    import time
    from grove.helper import SlotHelper

    sh = SlotHelper(SlotHelper.GPIO)
    pin = sh.argv2pin()

    pir_motion_wrapper = {
        'high-enable' : True,
        'direction'   : GPIO.IN,
        'status-attr' : "has_motion",
        'enable-attr' : "nothing",
        'disable-attr': "nothing"
    }

    pir_motion = GPIOWrapper(pin, pir_motion_wrapper)
    while True:
        if pir_motion.has_motion():
            print("Hi, people is moving")
        else:
            print("Watching")
        time.sleep(1)

if __name__ == '__main__':
    main()

