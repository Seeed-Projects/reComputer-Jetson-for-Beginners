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
from grove.led.one_led import OneLed
from rpi_ws281x import *

class OneLedTypedWs2812(OneLed):
    def __init__(self, pin):
        ws2812_pins = { 12:0, 13:1, 18:0, 19:1}
        if not pin in ws2812_pins.keys():
            print("OneLedTypedWs2812: pin {} could not used with WS2812".format(pin))
            return
        super(OneLedTypedWs2812, self).__init__(pin)

        # LED strip configuration:
        LED_COUNT      = 1       # Number of LED pixels.
        LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
        LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
        LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
        LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
        LED_CHANNEL    = ws2812_pins.get(pin) # set to '1' for GPIOs 13, 19, 41, 45 or 53

        # Create PixelStrip object with appropriate configuration.
        self.strip = PixelStrip(LED_COUNT, pin, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
        # Intialize the library (must be called once before other functions).
        self.strip.begin()

    def _lighton(self, on):
        if not on:
            r,g,b = 0,0,0
        else:
            r = self._bright * self._r / OneLed.MAX_BRIGHT
            g = self._bright * self._g / OneLed.MAX_BRIGHT
            b = self._bright * self._b / OneLed.MAX_BRIGHT
        self.strip.setPixelColor(0, Color(r,g,b))
        self.strip.show()

