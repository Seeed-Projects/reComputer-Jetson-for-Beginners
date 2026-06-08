#!/usr/bin/env python
#
# This library is for Grove - Recorder v3.0(https://www.seeedstudio.com/Grove-Recorder-v3.0-p-2709.html)
#
# This is the library for Grove Base Hat which used to connect grove sensors for raspberry pi.
#

'''
## License

The MIT License (MIT)

Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
Copyright (C) 2018 Seeed Technology Co.,Ltd. 

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
from RPi import GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class GroveRecorder():
    def __init__(self, play_pin, record_pin):
        self.play_pin = play_pin
        self.record_pin = record_pin

        GPIO.setup(self.play_pin, GPIO.OUT)
        GPIO.setup(self.record_pin, GPIO.OUT)
        GPIO.output(self.play_pin, 1)
        GPIO.output(self.record_pin, 1)

    def record(self, duration):
        GPIO.output(self.record_pin, 0)
        time.sleep(duration)
        GPIO.output(self.record_pin, 1)

    def play(self):
        GPIO.output(self.play_pin, 0)
        time.sleep(.5)
        GPIO.output(self.play_pin, 1)


Grove = GroveRecorder

def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.GPIO)
    pin = sh.argv2pin(" [record_duration]")

    import sys
    duration = 3
    if len(sys.argv) >= 3:
        duration = int(sys.argv[2])

    import time
    device = GroveRecorder(pin, pin + 1)
    print("Start recording for {} seconds".format(duration))
    device.record(duration)
    time.sleep(1)
    print("Start playing..")
    device.play()

if __name__ == '__main__':
    main()
