#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
# Port to support Grove - WS2813 RGB LED Strip Waterproff - XXX LED/m
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2018  Seeed Technology Co.,Ltd.
#
# NeoPixel library strandtest example
# Author: Tony DiCola (tony@tonydicola.com)
#
# Direct port of the Arduino NeoPixel library strandtest example.  Showcases
# various animations on a strip of NeoPixels.
'''
This is the code for
    - `Grove - WS2813 RGB LED Strip Waterproof - 30 LED/m -1m   <https://www.seeedstudio.com/Grove-WS2813-RGB-LED-Strip-Waterproof-30-LED-m-1m-p-3124.html>`_
    - `Grove - WS2813 RGB LED Strip Waterproof - 60 LED/m - 1m  <https://www.seeedstudio.com/Grove-WS2813-RGB-LED-Strip-Waterproof-60-LED-m-1m-p-3126.html>`_
    - `Grove - WS2813 RGB LED Strip Waterproof - 144 LED/m - 1m <https://www.seeedstudio.com/Grove-WS2813-RGB-LED-Strip-Waterproof-144-LED-m-1m-p-3127.html>`_

Examples:

    .. code-block:: python

        import time
        from rpi_ws281x import Color
        from grove.grove_ws2813_rgb_led_strip import GroveWS2813RgbStrip

        # connect to pin 12(slot PWM)
        PIN   = 12
        # For Grove - WS2813 RGB LED Strip Waterproof - 30 LED/m
        # there is 30 RGB LEDs.
        COUNT = 30
        strip = GroveWS2813RgbStrip(PIN, COUNT)

        # Define functions which animate LEDs in various ways.
        def colorWipe(strip, color, wait_ms=50):
            """Wipe color across display a pixel at a time."""
            for i in range(strip.numPixels()):
                strip.setPixelColor(i, color)
                strip.show()
                time.sleep(wait_ms/1000.0)

        print ('Color wipe animations.')
        colorWipe(strip, Color(255, 0, 0))  # Red wipe
        colorWipe(strip, Color(0, 255, 0))  # Blue wipe
        colorWipe(strip, Color(0, 0, 255))  # Green wipe
'''

__all__ = ['GroveWS2813RgbStrip', 'PixelStrip', 'Color']

import time
from rpi_ws281x import PixelStrip, Color

# LED strip configuration
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)

class GroveWS2813RgbStrip(PixelStrip):
    '''
    Wrapper Class for Grove - WS2813 RGB LED Strip Waterproof - XXX LED/m

    Args:
        pin(int)  : 12, 18 for RPi
        count(int): strip LEDs count
        brightness(int): optional, set to 0 for darkest and 255 for brightest, default 255
    '''
    def __init__(self, pin, count, brightness = None):
        ws2812_pins = { 12:0, 13:1, 18:0, 19:1}
        if not pin in ws2812_pins.keys():
            print("OneLedTypedWs2812: pin {} could not used with WS2812".format(pin))
            return
        channel = ws2812_pins.get(pin)

        if brightness is None:
            brightness = LED_BRIGHTNESS

        # Create PixelStrip object with appropriate configuration.
        super(GroveWS2813RgbStrip, self).__init__(
            count,
            pin,
            LED_FREQ_HZ,
            LED_DMA,
            LED_INVERT,
            brightness,
            channel
        )

        # Intialize the library (must be called once before other functions).
        self.begin()


# Define functions which animate LEDs in various ways.
def colorWipe(strip, color, wait_ms=50):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()
        time.sleep(wait_ms/1000.0)

def theaterChase(strip, color, wait_ms=50, iterations=10):
    """Movie theater light style chaser animation."""
    for j in range(iterations):
        for q in range(3):
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i+q, color)
            strip.show()
            time.sleep(wait_ms/1000.0)
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i+q, 0)

def wheel(pos):
    """Generate rainbow colors across 0-255 positions."""
    if pos < 85:
        return Color(pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return Color(255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return Color(0, pos * 3, 255 - pos * 3)

def rainbow(strip, wait_ms=20, iterations=1):
    """Draw rainbow that fades across all pixels at once."""
    for j in range(256*iterations):
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, wheel((i+j) & 255))
        strip.show()
        time.sleep(wait_ms/1000.0)

def rainbowCycle(strip, wait_ms=20, iterations=5):
    """Draw rainbow that uniformly distributes itself across all pixels."""
    for j in range(256*iterations):
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, wheel((int(i * 256 / strip.numPixels()) + j) & 255))
        strip.show()
        time.sleep(wait_ms/1000.0)

def theaterChaseRainbow(strip, wait_ms=50):
    """Rainbow movie theater light style chaser animation."""
    for j in range(256):
        for q in range(3):
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i+q, wheel((i+j) % 255))
            strip.show()
            time.sleep(wait_ms/1000.0)
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i+q, 0)


def main():
    from grove import helper
    from grove.helper import helper
    helper.root_check()

    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.PWM)
    pin = sh.argv2pin(" [led-count]")

    import sys
    count = 30
    if len(sys.argv) >= 3:
        count = int(sys.argv[2])

    strip = GroveWS2813RgbStrip(pin, count)

    print('Press Ctrl-C to quit.')
    try:
        while True:
            print ('Color wipe animations.')
            colorWipe(strip, Color(255, 0, 0))  # Red wipe
            colorWipe(strip, Color(0, 255, 0))  # Blue wipe
            colorWipe(strip, Color(0, 0, 255))  # Green wipe
            print ('Theater chase animations.')
            theaterChase(strip, Color(127, 127, 127))  # White theater chase
            theaterChase(strip, Color(127,   0,   0))  # Red theater chase
            theaterChase(strip, Color(  0,   0, 127))  # Blue theater chase
            print ('Rainbow animations.')
            rainbow(strip)
            rainbowCycle(strip)
            theaterChaseRainbow(strip)

    except KeyboardInterrupt:
        # clear all leds when exit
        colorWipe(strip, Color(0,0,0), 10)


if __name__ == '__main__':
    main()

