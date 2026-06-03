#!/usr/bin/env python
#
# GrovePi Library for using the Grove - I2C Motor Driver(http://www.seeedstudio.com/depot/Grove-I2C-Motor-Driver-p-907.html)
#
# The GrovePi connects the Raspberry Pi and Grove sensors.  You can learn more about GrovePi here:  http://www.dexterindustries.com/GrovePi
#
# Have a question about this library?  Ask on the forums here:  http://forum.dexterindustries.com/c/grovepi
#
# 2018-07-04 
# Modified by Seeed Studio, changed smbus module to smbus2

'''
## License

The MIT License (MIT)

GrovePi for the Raspberry Pi: an open source platform for connecting Grove Sensors to the Raspberry Pi.
Copyright (C) 2017  Dexter Industries

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
from __future__ import division
from grove.i2c import Bus
import time


class MotorDriver(object):
	__MotorSpeedSet             = 0x82
	__PWMFrequenceSet           = 0x84
	__DirectionSet              = 0xaa
	__MotorSetA                 = 0xa1
	__MotorSetB                 = 0xa5
	__Nothing                   = 0x01
	__EnableStepper             = 0x1a
	__UnenableStepper           = 0x1b
	__Stepernu                  = 0x1c
	I2CAddr                     = 0x0f  #Set the address of the I2CMotorDriver
	SPEED_MAX                   = 100

	def __init__(self,address=0x0f):
		self.I2CAddr = address
		self.bus = Bus()

	def __del__(self):
		self.set_speed(0, 0)

	#Maps speed from 0-100 to 0-255
	def _map_vals(self,value, leftMin, leftMax, rightMin, rightMax):
		#http://stackoverflow.com/questions/1969240/mapping-a-range-of-values-to-another
		# Figure out how 'wide' each range is
		leftSpan = leftMax - leftMin
		rightSpan = rightMax - rightMin

		# Convert the left range into a 0-1 range (float)
		valueScaled = float(value - leftMin) / float(leftSpan)

		# Convert the 0-1 range into a value in the right range.
		return int(rightMin + (valueScaled * rightSpan))
		
	#Set motor speed
	def set_speed(self, speed1 = 0, speed2 = 0):
		s1 = self._map_vals(speed1, 0, 100, 0, 255)
		s2 = self._map_vals(speed2, 0, 100, 0, 255)
		self.bus.write_i2c_block_data(self.I2CAddr, self.__MotorSpeedSet, [s1, s2])
		time.sleep(.02)
	
	#Set motor direction
	def set_dir(self, clock_wise1 = True, clock_wise2 = True):
		dir1 = 0b10 if clock_wise1 else 0b01
		dir2 = 0b10 if clock_wise2 else 0b01
		dir = (dir2 << 2) | dir1
		self.bus.write_i2c_block_data(self.I2CAddr, self.__DirectionSet, [dir, 0])
		time.sleep(.02)

Grove = MotorDriver

def main():
	print("Make sure I2C-Motor-Driver inserted")
	print("  in one I2C slot of Grove-Base-Hat")
	motor = MotorDriver()
	while True:
		motor.set_speed(MotorDriver.SPEED_MAX, MotorDriver.SPEED_MAX)
		motor.set_dir(True, True)
		time.sleep(2)
		motor.set_speed(MotorDriver.SPEED_MAX * 0.7, MotorDriver.SPEED_MAX * 0.7)
		motor.set_dir(False, False)
		time.sleep(2)

if __name__ == '__main__':
	main()

