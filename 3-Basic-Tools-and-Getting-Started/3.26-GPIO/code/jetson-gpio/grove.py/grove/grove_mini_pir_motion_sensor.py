#!/usr/bin/env python
#
# This library is for
#   Grove - mini PIR motion sensor(https://www.seeedstudio.com/Grove-mini-PIR-motion-sensor-p-2930.html)
#   Grove - PIR Mtion Sensor(https://www.seeedstudio.com/Grove-PIR-Motion-Sensor-p-802.html)
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
import time
from grove.gpio import GPIO


class GroveMiniPIRMotionSensor(GPIO):
    def __init__(self, pin):
        super(GroveMiniPIRMotionSensor, self).__init__(pin, GPIO.IN)
        self._on_detect = None

    @property
    def on_detect(self):
        return self._on_detect

    @on_detect.setter
    def on_detect(self, callback):
        if not callable(callback):
            return

        if self.on_event is None:
            self.on_event = self._handle_event

        self._on_detect = callback

    def _handle_event(self, pin, value):
        if value:
            if callable(self._on_detect):
                self._on_detect()

Grove = GroveMiniPIRMotionSensor


def main():
    from grove.helper import SlotHelper
    sh = SlotHelper(SlotHelper.GPIO)
    pin = sh.argv2pin()

    pir = GroveMiniPIRMotionSensor(pin)

    def callback():
        print('Motion detected.')

    pir.on_detect = callback

    while True:
        time.sleep(1)


if __name__ == '__main__':
    main()
