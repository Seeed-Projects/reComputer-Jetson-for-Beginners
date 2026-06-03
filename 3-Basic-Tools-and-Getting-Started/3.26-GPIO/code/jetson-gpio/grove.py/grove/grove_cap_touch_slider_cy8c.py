#!/usr/bin/env python
#
# This is the code for Grove - Capacitive Touch Slide Sensor(CY8C4014LQI).
# (https://www.seeedstudio.com/Grove-Capacitive-Touch-Slider-Sensor-CY8C4014LQ-p-3183.html)
# which is a multifunctional touch sensor.
# You can touch the two buttons or slide on the 5-segment slider,
# each touch pad has a corresponding LED to indicate the touch status.
#
'''
## License
Author: Downey
The MIT License (MIT)

Grove touch sensor CY8C for the Raspberry Pi, used to connect grove sensors.
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

TOUCH_SENSOR_CY8C_DEFAULT_IIC_ADDR        = 0X08
BUTTON_REG_ADDR                           = 0X00
TOUCH_SLIDER_REG_ADDR                     = 0X01

UNPRESSED                                 = 0
PRESSED                                   = 1

class GroveTouchSliderCy8c():
    
    left_button_press_flag = UNPRESSED
    right_button_press_flag = UNPRESSED

    def __init__(self,bus_num = 1, addr = TOUCH_SENSOR_CY8C_DEFAULT_IIC_ADDR):
        self.bus = Bus(bus_num)
        self.addr = addr
    
    def read_sensor_button_value(self):
        button_value = self.bus.read_byte_data(self.addr,BUTTON_REG_ADDR)
        return button_value

    def read_sensor_slider_value(self):
        slider_value = self.bus.read_byte_data(self.addr,TOUCH_SLIDER_REG_ADDR)
        return slider_value
    
    def parse_and_print_result(self,button_value,slider_value):

        '''
        if(0 == button_value):
            if(PRESSED == self.left_button_press_flag):
                print("Left button is released")
                self.left_button_press_flag = UNPRESSED
            if(PRESSED == self.right_button_press_flag):
                print("Right button is released")
                self.right_button_press_flag = UNPRESSED
        '''
        if(0x01 ==  button_value&0x1 ):
            if(UNPRESSED == self.left_button_press_flag): 
                print("Left button is pressed")
                self.left_button_press_flag = PRESSED
        elif(0 == button_value&0x1):
            if(PRESSED == self.left_button_press_flag):
                print("Left button is released")
                self.left_button_press_flag = UNPRESSED
        if(0x02 == button_value&0x2):
            if(UNPRESSED == self.right_button_press_flag): 
                print("Right button is pressed")
                self.right_button_press_flag = PRESSED
        elif(0 == button_value&0x2):
            if(PRESSED == self.right_button_press_flag):
                print("Right button is released")
                self.right_button_press_flag = UNPRESSED
        if(0 != slider_value):
            print("Slider is pressed,value is %s" %(slider_value))
    def listen_sensor_status(self):
        while True:
            button_value = self.read_sensor_button_value()
            slider_value = self.read_sensor_slider_value()
            self.parse_and_print_result(button_value,slider_value)
            time.sleep(.05)


CY8C = GroveTouchSliderCy8c()

def main():
     CY8C.listen_sensor_status()   


if __name__ == '__main__':
    main()
        
