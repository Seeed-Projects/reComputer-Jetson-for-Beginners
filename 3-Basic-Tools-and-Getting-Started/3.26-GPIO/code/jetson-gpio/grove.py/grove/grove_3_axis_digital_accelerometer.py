#!/usr/bin/env python
#
# This code is for Grove - 3-Axis Digital Accelerometer(+/-400g)
# (https://www.seeedstudio.com/Grove-3-Axis-Digital-Accelerometer-400-p-1897.html)
#
# which is a low power high performance 3-axis linear accelerometer belonging
# to the "nano" family, with digital I2C serial interface standard output.
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

from __future__ import print_function
from grove.i2c import Bus
import time

# I2C address of the device
H3LIS331DL_DEFAULT_ADDRESS			= 0x19

# H3LIS331DL Register Map
H3LIS331DL_REG_WHOAMI					= 0x0F # Who Am I Register
H3LIS331DL_REG_CTRL1					= 0x20 # Control Register-1
H3LIS331DL_REG_CTRL2					= 0x21 # Control Register-2
H3LIS331DL_REG_CTRL3					= 0x22 # Control Register-3
H3LIS331DL_REG_CTRL4					= 0x23 # Control Register-4
H3LIS331DL_REG_CTRL5					= 0x24 # Control Register-5
H3LIS331DL_REG_REFERENCE				= 0x26 # Reference
H3LIS331DL_REG_STATUS					= 0x27 # Status Register
H3LIS331DL_REG_OUT_X_L					= 0x28 # X-Axis LSB
H3LIS331DL_REG_OUT_X_H					= 0x29 # X-Axis MSB
H3LIS331DL_REG_OUT_Y_L					= 0x2A # Y-Axis LSB
H3LIS331DL_REG_OUT_Y_H					= 0x2B # Y-Axis MSB
H3LIS331DL_REG_OUT_Z_L					= 0x2C # Z-Axis LSB
H3LIS331DL_REG_OUT_Z_H					= 0x2D # Z-Axis MSB

# Accl Datarate configuration
H3LIS331DL_ACCL_PM_PD					= 0x00 # Power down Mode
H3LIS331DL_ACCL_PM_NRMl					= 0x20 # Normal Mode
H3LIS331DL_ACCL_PM_0_5					= 0x40 # Low-Power Mode, ODR = 0.5Hz
H3LIS331DL_ACCL_PM_1					= 0x60 # Low-Power Mode, ODR = 1Hz
H3LIS331DL_ACCL_PM_2					= 0x80 # Low-Power Mode, ODR = 2Hz
H3LIS331DL_ACCL_PM_5					= 0xA0 # Low-Power Mode, ODR = 5Hz
H3LIS331DL_ACCL_PM_10					= 0xC0 # Low-Power Mode, ODR = 10Hz
H3LIS331DL_ACCL_DR_50					= 0x00 # ODR = 50Hz
H3LIS331DL_ACCL_DR_100					= 0x08 # ODR = 100Hz
H3LIS331DL_ACCL_DR_400					= 0x10 # ODR = 400Hz
H3LIS331DL_ACCL_DR_1000					= 0x18 # ODR = 1000Hz

# Accl Data update & Axis configuration
H3LIS331DL_ACCL_LPEN					= 0x00 # Normal Mode, Axis disabled
H3LIS331DL_ACCL_XAXIS					= 0x04 # X-Axis enabled
H3LIS331DL_ACCL_YAXIS					= 0x02 # Y-Axis enabled
H3LIS331DL_ACCL_ZAXIS					= 0x01 # Z-Axis enabled

# Acceleration Full-scale selection
H3LIS331DL_ACCL_BDU_CONT				= 0x00 # Continuous update, Normal Mode, 4-Wire Interface, LSB first
H3LIS331DL_ACCL_BDU_NOT_CONT			= 0x80 # Output registers not updated until MSB and LSB reading
H3LIS331DL_ACCL_BLE_MSB					= 0x40 # MSB first
H3LIS331DL_ACCL_RANGE_400G				= 0x30 # Full scale = +/-400g
H3LIS331DL_ACCL_RANGE_200G				= 0x10 # Full scale = +/-200g
H3LIS331DL_ACCL_RANGE_100G				= 0x00 # Full scale = +/-100g
H3LIS331DL_ACCL_SIM_3					= 0x01 # 3-Wire Interface
H3LIS331DL_RAW_DATA_MAX					= 65536

H3LIS331DL_DEFAULT_RANGE = H3LIS331DL_ACCL_RANGE_100G
H3LIS331DL_SCALE_FS = H3LIS331DL_RAW_DATA_MAX / 4 / ((H3LIS331DL_DEFAULT_RANGE >> 4) + 1)

class H3LIS331DL(object):
	def __init__ (self, address=H3LIS331DL_DEFAULT_ADDRESS):
		self._addr = address
		self._bus  = Bus()
		self.select_datarate()
		self.select_data_config()
	
	def select_datarate(self):
		"""Select the data rate of the accelerometer from the given provided values"""
		DATARATE_CONFIG = (H3LIS331DL_ACCL_PM_NRMl | H3LIS331DL_ACCL_DR_50 | H3LIS331DL_ACCL_XAXIS | H3LIS331DL_ACCL_YAXIS | H3LIS331DL_ACCL_ZAXIS)
		self._bus.write_byte_data(self._addr, H3LIS331DL_REG_CTRL1, DATARATE_CONFIG)
	
	def select_data_config(self):
		"""Select the data configuration of the accelerometer from the given provided values"""
		DATA_CONFIG = (H3LIS331DL_DEFAULT_RANGE | H3LIS331DL_ACCL_BDU_CONT)
		self._bus.write_byte_data(self._addr, H3LIS331DL_REG_CTRL4, DATA_CONFIG)
	
	def read_accl(self):
		"""Read data back from H3LIS331DL_REG_OUT_X_L(0x28), 2 bytes
		X-Axis Accl LSB, X-Axis Accl MSB"""
		data0 = self._bus.read_byte_data(self._addr, H3LIS331DL_REG_OUT_X_L)
		data1 = self._bus.read_byte_data(self._addr, H3LIS331DL_REG_OUT_X_H)
		
		xAccl = data1 * 256 + data0
		if xAccl > H3LIS331DL_RAW_DATA_MAX / 2:
			xAccl -= H3LIS331DL_RAW_DATA_MAX
		
		"""Read data back from H3LIS331DL_REG_OUT_Y_L(0x2A), 2 bytes
		Y-Axis Accl LSB, Y-Axis Accl MSB"""
		data0 = self._bus.read_byte_data(self._addr, H3LIS331DL_REG_OUT_Y_L)
		data1 = self._bus.read_byte_data(self._addr, H3LIS331DL_REG_OUT_Y_H)
		
		yAccl = data1 * 256 + data0
		if yAccl > H3LIS331DL_RAW_DATA_MAX / 2 :
			yAccl -= H3LIS331DL_RAW_DATA_MAX
		
		"""Read data back from H3LIS331DL_REG_OUT_Z_L(0x2C), 2 bytes
		Z-Axis Accl LSB, Z-Axis Accl MSB"""
		data0 = self._bus.read_byte_data(self._addr, H3LIS331DL_REG_OUT_Z_L)
		data1 = self._bus.read_byte_data(self._addr, H3LIS331DL_REG_OUT_Z_H)
		
		zAccl = data1 * 256 + data0
		if zAccl > H3LIS331DL_RAW_DATA_MAX / 2 :
			zAccl -= H3LIS331DL_RAW_DATA_MAX
		
		return {'x' : xAccl, 'y' : yAccl, 'z' : zAccl}

def main():
	h3lis331dl = H3LIS331DL()
	while True:
		h3lis331dl.select_datarate()
		h3lis331dl.select_data_config()
		time.sleep(0.2)
		accl = h3lis331dl.read_accl()
		print("Raw:    X = {0:6}   Y = {1:6}   Z = {2:6}"
			.format(accl['x'], accl['y'], accl['z']))
		print("Accel: AX = {0:6.3}g AY = {1:6.3}g AZ = {2:6.3}g"
			.format(accl['x'] / H3LIS331DL_SCALE_FS, accl['y'] / H3LIS331DL_SCALE_FS, accl['z'] / H3LIS331DL_SCALE_FS))
		time.sleep(.5)

if __name__ == '__main__':
    main()