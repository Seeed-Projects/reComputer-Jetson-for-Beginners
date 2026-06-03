#!/usr/bin/env python
#
# This is the code for Grove - UV Sensor
# (https://www.seeedstudio.com/Grove-UV-Sensor-p-1540.html)
#
# This is the library used with Grove Base Hat for RPi.

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
from grove.i2c import Bus
import time


# I2C address of the device
VEML6070_DEFAULT_ADDRESS				= 0x38

# VEML6070 Command Set
VEML6070_CMD_ACK_DISABLE				= 0x00 # Acknowledge Disable
VEML6070_CMD_ACK_ENABLE					= 0x20 # Acknowledge Enable
VEML6070_CMD_ACK_THD_102				= 0x00 # Acknowledge threshold 102 Steps
VEML6070_CMD_ACK_THD_145				= 0x10 # Acknowledge threshold 145 Steps
VEML6070_CMD_IT_1_2T					= 0x00 # Integration time = 1/2T
VEML6070_CMD_IT_1T						= 0x04 # Integration time = 1T
VEML6070_CMD_IT_2T						= 0x08 # Integration time = 2T
VEML6070_CMD_IT_4T						= 0x0C # Integration time = 4T
VEML6070_CMD_RESERVED					= 0x02 # Reserved, Set to 1
VEML6070_CMD_SD_DISABLE					= 0x00 # Shut-down Disable
VEML6070_CMD_SD_ENABLE					= 0x01 # Shut-down Enable
VEML6070_CMD_READ_LSB					= 0x38 # Read LSB of the data
VEML6070_CMD_READ_MSB					= 0x39 # Read MSB of the data

class VEML6070():
	def __init__(self, address = VEML6070_DEFAULT_ADDRESS):
		self._addr = address
		self._bus  = Bus()
		self.write_command()
	
	def write_command(self):
		"""Select the UV light command from the given provided values"""
		COMMAND_CONFIG = (VEML6070_CMD_ACK_DISABLE | VEML6070_CMD_IT_1_2T | VEML6070_CMD_SD_DISABLE | VEML6070_CMD_RESERVED)
		self._bus.write_byte(VEML6070_DEFAULT_ADDRESS, COMMAND_CONFIG)
	
	def read_uvlight(self):
		"""Read data back VEML6070_CMD_READ_MSB(0x73) and VEML6070_CMD_READ_LSB(0x71), uvlight MSB, uvlight LSB"""
		data0 = self._bus.read_byte(VEML6070_CMD_READ_MSB)
		data1 = self._bus.read_byte(VEML6070_CMD_READ_LSB)
		
		# Convert the data
		uvlight = data0 * 256 + data1
		
		return {'u' : uvlight}


def main():

	veml6070 = VEML6070()

	## Exit handlers ##
	# This function stops python from printing a stacktrace when you hit control-C
	def SIGINTHandler(signum, frame):
		raise SystemExit

	# This function lets you run code on exit, including functions from abpdrrt005pg2a5
	def exitHandler():
		print("Exiting")
		sys.exit(0)
	
	while True:
		light = veml6070.read_uvlight()
		print("UV Value: {0}".format(light['u']))
		print(" *********************************** ")
		time.sleep(1)

if __name__ == '__main__':
	main()