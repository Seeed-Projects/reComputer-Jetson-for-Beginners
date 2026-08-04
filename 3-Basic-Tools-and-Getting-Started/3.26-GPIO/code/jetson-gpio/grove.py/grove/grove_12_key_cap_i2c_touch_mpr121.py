#!/usr/bin/env python
#
# This is the code for Grove - 12 Key Capacitive I2C Touch Sensor V2(MPR121).
# (https://www.seeedstudio.com/Grove-12-Key-Capacitive-I2C-Touch-Sensor-V2-MPR12-p-3141.html)
# which is is a multichannel proximity capacitive touch sensor.
# It's a 3-in-1 module with the following features:
#   Capacitance Sensing, Touch Sensing, and Proximity Sensing.
#
'''
## License
Author: Downey
The MIT License (MIT)

Grove 12 channel touch sensor MPR121 for the Raspberry Pi, used to connect grove sensors.
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


TOUCH_SENSOR_DEFAULT_ADDR                 = 0x5b

MODE_CONFIG_REG_ADDR                      = 0x5e
GLOBAL_PARAM_REG_ADDR_L                   = 0x5c
TOUCH_STATUS_REG_ADDR_L                   = 0x00
SET_DEBOUNCE_REG_ADDR                     = 0x5b

FILTERED_DATA_REG_START_ADDR_L            = 0x04
CHANNEL_NUM                               = 12

STOP_MODE                                 = 0
NORMAL_MODE                               = 0x3c

class Grove12KeyCapTouchMpr121():
    def __init__(self,bus_num = 1,addr = TOUCH_SENSOR_DEFAULT_ADDR):
        self.bus = Bus(bus_num)
        self.addr = addr
        self.threshold = 0
        self.touch_flag = [0]*CHANNEL_NUM

    def sensor_init(self):
        self._set_mode(STOP_MODE)
        data = [0x23,0x10]
        self._set_global_param(data)
        self._set_debounce(0x22)
        self._set_mode(NORMAL_MODE)

    def set_threshold(self,threshold):
        self.threshold = threshold

    def wait_for_ready(self):
        time.sleep(.2)

    def _set_mode(self,mode):
        self.bus.write_byte_data(self.addr,MODE_CONFIG_REG_ADDR,mode)
    
    def _set_global_param(self,data):
        self.bus.write_i2c_block_data(self.addr,GLOBAL_PARAM_REG_ADDR_L,data)
    
    def _set_debounce(self,data):
        self.bus.write_byte_data(self.addr,SET_DEBOUNCE_REG_ADDR,data)

    def _check_status_register(self):
        data_status = self.bus.read_i2c_block_data(self.addr,TOUCH_STATUS_REG_ADDR_L,2)
        return data_status
    
    def get_filtered_touch_data(self,sensor_status):
        result_value = []
        for i in range(CHANNEL_NUM):
            time.sleep(.01)
            if(sensor_status & (1<<i)):
                channel_data = self.bus.read_i2c_block_data(self.addr,FILTERED_DATA_REG_START_ADDR_L+2*i,2)
                result_value.append(channel_data[0] | channel_data[1]<<8 )
            else:
                result_value.append(0)
        return result_value

    def listen_sensor_status(self):
        data = self._check_status_register()
        touch_status = data[0] | (data[1]<<8) 
        touch_result_value = self.get_filtered_touch_data(touch_status)

        for i in range(CHANNEL_NUM):
            if(touch_result_value[i] < self.threshold ):
                touch_result_value[i] = 0
        return touch_result_value
    
    def parse_and_print_result(self,result):
        for i in range(CHANNEL_NUM):
            if(result[i] != 0):
                if(0 == self.touch_flag[i]):
                    self.touch_flag[i] = 1
                    print("Channel %d is pressed,value is %d" %(i,result[i]))
            else:
                if(1 == self.touch_flag[i]):
                    self.touch_flag[i] = 0
                    print("Channel %d is released,value is %d" %(i,result[i]))
        


mpr121 = Grove12KeyCapTouchMpr121() 
def main():
    mpr121.sensor_init()
    mpr121.set_threshold(0x60)
    mpr121.wait_for_ready()
    while 1:
        result = mpr121.listen_sensor_status()
        mpr121.parse_and_print_result(result)
        time.sleep(.1)

if __name__  == '__main__':
    main()