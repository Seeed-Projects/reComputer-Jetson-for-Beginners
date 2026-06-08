#!/usr/bin/env python
#
# This is the library for Grove Base Hat.
#
# OneLed Classes
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
import threading
from grove.gpio import GPIO

class OneLed(object):
    MAX_BRIGHT = 100

    def __init__(self, pin):
        self._light = False
        self._bright = self.MAX_BRIGHT
        self.__blink_on = 0.0
        self.__blink_off = 0.0
        self.__thrd = None
        self.__thr_exit = False
        self._r, self._g, self._b = 255,255,255

    def __thrd_exit(self):
        if not self.__thrd is None:
            self.__thr_exit = True
            self.__thrd.join()
            self.__thrd = None

    def __del__(self):
        self.__thrd_exit()

    def __thrd_blink(self):
        while not self.__thr_exit:
            self._lighton(True)
            time.sleep(self.__blink_on)
            self._lighton(False)
            time.sleep(self.__blink_off)

    def blink(self, on = 0.5, off = 0.5):
        self.__blink_on = on
        self.__blink_off = off
        if on < 0.001 and off < 0.001:
            self.__thrd_exit()
            return

        if not self.__thrd is None:
            return

        self.__thr_exit = False
        self.__thrd = threading.Thread( \
                target = OneLed.__thrd_blink, \
                args = (self,))
        self.__thrd.setDaemon(True)
        self.__thrd.start()

    # To be derived
    def _lighton(self, b = True):
        pass

    def light(self, on = None):
        if type(on) != bool:
            return self._light
        self.__thrd_exit()
        self._light = on
        self._lighton(self._light)
        return self._light

    # To be derived
    def _setcolor(self, r, g, b):
        return False

    # Set color
    def color(self, r = None, g = 0, b = 0):
        if type(r) != int:
            return self._r, self._g, self._b
        if self._setcolor(r, g, b):
            self._r, self._g, self, _b = r, g, b
        return self._r, self._g, self._b

    @property
    def brightness(self):
        return self._bright

    # bright 0(turn off) - 100(max bright)
    @brightness.setter
    def brightness(self, bright):
        self._bright = bright


class OneLedTypedGpio(OneLed):
    def __init__(self, pin, high_enable = True):
        super(OneLedTypedGpio, self).__init__(pin)
        self.__high_en = high_enable
        self.__gpio = GPIO(pin, GPIO.OUT)

    def _lighton(self, b = True):
        v = self.__high_en == bool(b) and bool(self._bright)
        # print("write {} {}".format(self._bright, int(v)))
        self.__gpio.write(int(v))

