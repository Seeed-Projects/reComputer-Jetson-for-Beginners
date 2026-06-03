#!/usr/bin/env python
#
# Library for interacting with Grove - HP20x sensor (used to measure temperature, pressure and altitude)
#
# This is the library for Grove Base Hat which used to connect grove sensors for raspberry pi.
#

'''
## License

The MIT License (MIT)

Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
Copyright (C) [Your Company Name or Relevant Party] 

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

# Class for interacting with the HP20x sensor
class HP20x:
    def __init__(self):
        # Initialize the I2C bus on Raspberry Pi (bus 1)
        self.bus = Bus()
        # I2C address of the HP206F sensor, may need adjustment based on actual situation
        self.address = 0x76

        # I2C device ID when CSB PIN is at VDD level (address is 0x76)
        self.HP20X_I2C_DEV_ID = (0xEC) >> 1
        # I2C device ID when CSB PIN is at GND level (address is 0x77)
        self.HP20X_I2C_DEV_ID2 = (0XEE) >> 1
        # Soft reset command for the HP20x sensor
        self.HP20X_SOFT_RST = 0x06
        # Write conversion command for the HP20x sensor
        self.HP20X_WR_CONVERT_CMD = 0x40
        # Different oversampling rate (OSR) configurations for conversion
        self.HP20X_CONVERT_OSR4096 = 0 << 2
        self.HP20X_CONVERT_OSR2048 = 1 << 2
        self.HP20X_CONVERT_OSR1024 = 2 << 2
        self.HP20X_CONVERT_OSR512 = 3 << 2
        self.HP20X_CONVERT_OSR256 = 4 << 2
        self.HP20X_CONVERT_OSR128 = 5 << 2

        # Commands for reading pressure, altitude, temperature, etc.
        self.HP20X_READ_P = 0x30  # Read pressure command
        self.HP20X_READ_A = 0x31  # Read altitude command
        self.HP20X_READ_T = 0x32  # Read temperature command
        self.HP20X_READ_PT = 0x10  # Read pressure and temperature command
        self.HP20X_READ_AT = 0x11  # Read altitude and temperature command
        self.HP20X_READ_CAL = 0X28  # RE-CAL ANALOG command

        # Write register mode for the HP20x sensor
        self.HP20X_WR_REG_MODE = 0xC0
        # Read register mode for the HP20x sensor
        self.HP20X_RD_REG_MODE = 0x80

        # Set the oversampling rate configuration
        self.OSR_CFG = self.HP20X_CONVERT_OSR1024
        # Conversion time corresponding to the oversampling rate (in milliseconds)
        self.OSR_ConvertTime = 25

    def begin(self):
        # Send a soft reset command to the HP20x sensor
        self.HP20X_IIC_WriteCmd(self.HP20X_SOFT_RST)
        # Wait for 0.1 seconds to ensure the reset operation is completed
        time.sleep(0.1)

    def isAvailable(self):
        # Check if the HP20x sensor is available by reading the register at address 0x0F
        return self.HP20X_IIC_ReadReg(0x0F)

    def ReadTemperature(self):
        # Send a conversion command with the specified oversampling rate configuration
        self.HP20X_IIC_WriteCmd(self.HP20X_WR_CONVERT_CMD | self.OSR_CFG)
        # Wait for the conversion time (converted to seconds)
        time.sleep(self.OSR_ConvertTime / 1000.0)
        # Read 3 bytes of raw temperature data from the sensor
        t_raw = self.bus.read_i2c_block_data(self.address, self.HP20X_READ_T, 3)
        # Combine the 3 bytes of data to form a single value
        t = t_raw[0] << 16 | t_raw[1] << 8 | t_raw[2]
        # Handle negative values using 2's complement
        if t & 0x800000:
            t |= 0xff000000
            us = (1 << 32)
            t = -1 * (us - t)
        # Return the temperature value in degrees Celsius (divided by 100)
        return t / 100.0

    def ReadPressure(self):
        # Send a conversion command with the specified oversampling rate configuration
        self.HP20X_IIC_WriteCmd(self.HP20X_WR_CONVERT_CMD | self.OSR_CFG)
        # Wait for the conversion time (converted to seconds)
        time.sleep(self.OSR_ConvertTime / 1000.0)
        # Read 3 bytes of raw pressure data from the sensor
        p_raw = self.bus.read_i2c_block_data(self.address, self.HP20X_READ_P, 3)
        # Combine the 3 bytes of data to form a single value
        p = p_raw[0] << 16 | p_raw[1] << 8 | p_raw[2]
        # Handle negative values using 2's complement
        if p & 0x800000:
            p |= 0xff000000
        # Return the pressure value in hectopascals (divided by 100)
        return p / 100.0

    def ReadAltitude(self):
        # Send a conversion command with the specified oversampling rate configuration
        self.HP20X_IIC_WriteCmd(self.HP20X_WR_CONVERT_CMD | self.OSR_CFG)
        # Wait for the conversion time (converted to seconds)
        time.sleep(self.OSR_ConvertTime / 1000.0)
        # Read 3 bytes of raw altitude data from the sensor
        a_raw = self.bus.read_i2c_block_data(self.address, self.HP20X_READ_A, 3)
        # Combine the 3 bytes of data to form a single value
        a = a_raw[0] << 16 | a_raw[1] << 8 | a_raw[2]
        # Handle negative values using 2's complement
        if a & 0x800000:
            a |= 0xff000000
            us = (1 << 32)
            a = -1 * (us - a)
        # Return the altitude value in meters (divided by 100)
        return a / 100.0

    def HP20X_IIC_WriteCmd(self, uCmd):
        # Write a command byte to the specified I2C address
        self.bus.write_byte(self.address, uCmd)

    def HP20X_IIC_ReadReg(self, bReg):
        # Read a byte from the specified register address
        return self.bus.read_byte_data(self.address, bReg | self.HP20X_RD_REG_MODE)


# Class representing the Kalman filter
class KalmanFilter:
    def __init__(self):
        # Process noise covariance
        self.q = 0.01
        # Measurement noise covariance
        self.r = 0.1
        # Initial estimated value
        self.x = 0
        # Initial estimated error covariance
        self.p = 1
        # Initial Kalman gain
        self.k = 0

    def Filter(self, measurement):
        # Prediction step: Update the estimated error covariance
        self.p = self.p + self.q
        # Update step: Calculate the Kalman gain
        self.k = self.p / (self.p + self.r)
        # Update step: Update the estimated value based on the measurement
        self.x = self.x + self.k * (measurement - self.x)
        # Update step: Update the estimated error covariance
        self.p = (1 - self.k) * self.p
        # Return the filtered estimated value
        return self.x


# Kalman filter for temperature data
t_filter = KalmanFilter()
# Kalman filter for pressure data
p_filter = KalmanFilter()
# Kalman filter for altitude data
a_filter = KalmanFilter()

# Create an instance of the HP20x sensor
hp20x = HP20x()


# Function to simulate the setup process
def setup():
    print("****HP20x_dev demo by seeed studio****\n")
    print("Calculation formula: H = [8.5(101325-P)]/100 \n")
    # Wait for 0.15 seconds after power-on to stabilize the voltage
    time.sleep(0.15)
    # Initialize the HP20x sensor
    hp20x.begin()
    # Wait for 0.1 seconds
    time.sleep(0.1)
    # Check if the HP20x sensor is available
    ret = hp20x.isAvailable()
    if ret:
        print("HP20x_dev is available.\n")
    else:
        print("HP20x_dev isn't available.\n")
    return ret


# Function to simulate the loop process
def loop(ret):
    if ret:
        while True:
            print("------------------\n")
            # Read the temperature value from the HP20x sensor
            temper = hp20x.ReadTemperature()
            print("Temper:")
            print(f"{temper}C.\n")
            print("Filter:")
            # Apply the Kalman filter to the temperature value
            print(f"{t_filter.Filter(temper)}C.\n")

            # Read the pressure value from the HP20x sensor
            pressure = hp20x.ReadPressure()
            print("Pressure:")
            print(f"{pressure}hPa.\n")
            print("Filter:")
            # Apply the Kalman filter to the pressure value
            print(f"{p_filter.Filter(pressure)}hPa\n")

            # Read the altitude value from the HP20x sensor
            altitude = hp20x.ReadAltitude()
            print("Altitude:")
            print(f"{altitude}m.\n")
            print("Filter:")
            # Apply the Kalman filter to the altitude value
            print(f"{a_filter.Filter(altitude)}m.\n")
            print("------------------\n")
            # Wait for 1 second before the next reading
            time.sleep(1)


if __name__ == "__main__":
    # Perform the setup process
    ret = setup()
    # Start the loop process if the sensor is available
    loop(ret)
    