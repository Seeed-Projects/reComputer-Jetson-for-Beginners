"""
License
The MIT License (MIT)

Copyright (C) 2020 Ville Laine, Aikuiskoulutus Taitaja

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
"""

from time import sleep, clock_gettime, CLOCK_REALTIME
from grove.i2c import Bus


class bme280:
    """
    Grove bme280 sensor control library.
    Objects:
        - set_mode(mode, t_sb)
        - set_oversampling(osrs_h, osrs_t, osrs_p)
        - set_filter(filter_coefficient)
        - set_spi(spi3w_en)
        - write_reset()
        - read_id()
        - read_status()
        - read_compensated_signals()
        - read_raw_signals()
        - set_pressure_calibration(level, pressure)
        - get_altitude(pressure)
    """

    # Constant Definitions
    # Modes
    MODE_SLEEP = 0b00
    MODE_FORCE = 0b01
    MODE_NORMAL = 0b11

    # Normal mode standby values
    t_sb_0_5 = 0b000
    t_sb_62_5 = 0b001
    t_sb_125 = 0b010
    t_sb_250 = 0b011
    t_sb_500 = 0b100
    t_sb_1000 = 0b101
    t_sb_10 = 0b110
    t_sb_20 = 0b111

    # IIR filter coefficient
    filter_0 = 0b000
    filter_2 = 0b001
    filter_4 = 0b010
    filter_8 = 0b011
    filter_16 = 0b100

    # SPI control
    SPI_ON = 0b01
    SPI_OFF = 0b00

    # Oversampling modes
    OVRS_x0 = 0b000
    OVRS_x1 = 0b001
    OVRS_x2 = 0b010
    OVRS_x4 = 0b011
    OVRS_x8 = 0b100
    OVRS_x16 = 0b101

    def __init__(self):
        # Rev 2 Pi, Pi 2 & Pi 3 uses bus 1,  Rev 1 Pi uses bus 0
        # bme280 default I2C Address is 0x76
        self.bus = Bus()
        self.address = 0x76
        self.mode = self.MODE_SLEEP
        self.t_sb = self.t_sb_1000
        self.last_meas = 0

        # Calibration data
        # Calibration data registers are 0x88-0xA1(25 bytes) and 0xE1-0xE7(7 bytes) total 32 bytes
        self.calib = [0x00]*32

        # Init raw data array and variables
        self.raw_data = [0x00]*8
        self.raw_temperature = 0x00000
        self.raw_pressure = 0x00000
        self.raw_humidity = 0x00000

        # Init parameters
        self.osrs_h = self.OVRS_x0
        self.osrs_t = self.OVRS_x0
        self.osrs_p = self.OVRS_x0

        # IIR filter and SPI interface are off by default
        self.filter = self.filter_0
        self.spi3w_en = self.SPI_OFF

        # Status word. Only has two significant bits bit0 = im_update and bit3 = measuring
        self.status = 0x00
        self.im_update = False
        self.measuring = False

        # Compensation values
        # Temperature trimming values
        self.dig_T1 = 0x0000
        self.dig_T2 = 0x0000
        self.dig_T3 = 0x0000
        # Pressure trimming params
        self.dig_P1 = 0x0000
        self.dig_P2 = 0x0000
        self.dig_P3 = 0x0000
        self.dig_P4 = 0x0000
        self.dig_P5 = 0x0000
        self.dig_P6 = 0x0000
        self.dig_P7 = 0x0000
        self.dig_P8 = 0x0000
        self.dig_P9 = 0x0000
        # Humidity trimming params
        self.dig_H1 = 0x00
        self.dig_H2 = 0x0000
        self.dig_H3 = 0x00
        self.dig_H4 = 0x0000
        self.dig_H5 = 0x0000
        self.dig_H6 = 0x00

        # Get the trimming values
        self.__read_calib()

        # Variables for compensated measurements
        self.temperature = 0x00000
        self.pressure = 0x00000
        self.humidity = 0x00000
        self.calibrated_temperature = 0x00000
        self.calibrated_pressure = 0x00000
        self.calibrated_humidity = 0x00000

        # Variables for measurement calibration
        self.calibration_temperature = 0
        self.calibration_pressure = 0
        self.calibration_humidity = 0

    #########################################
    # set bme280 operating mode             #
    #########################################
    def set_mode(self, mode=MODE_SLEEP, t_sb=t_sb_1000):
        # Writes ctrl_meas register with current temperature and pressure oversampling settings
        # Only changes the mode
        # If normal mode selected also sets the standby time in config register

        # Set class variables
        self.mode = mode
        self.t_sb = t_sb

        # If no measurements are enabled there is no point going into measurement
        if self.osrs_t + self.osrs_p + self.osrs_h == 0:
            print("No measurement enabled!\nSee set_oversampling()-function to enable measurement.")
            return 0

        try:
            # If normal mode set also t_sb(standby time)
            if self.mode == self.MODE_NORMAL:
                # Write normal mode standby time t_sb to config register
                self.__config(t_sb, self.filter, self.spi3w_en)
                # Write mode to ctr_meas register
                self.__ctrl_meas(self.osrs_t, self.osrs_p, self.mode)
            # Otherwise just change the mode in ctrl_meas register
            else:
                self.__ctrl_meas(self.osrs_t, self.osrs_p, self.mode)

            self.last_meas = clock_gettime(CLOCK_REALTIME)
            # Everything went well return 1
            return 1
        except Exception as e:
            # e = sys.exc_info()[0]
            print("<p>Error: %s</p>" % e)
            return 0

    #########################################################################
    # Set oversampling/enable measurement. OVRS_x0 disables the measurement #
    #########################################################################
    def set_oversampling(self, osrs_h=OVRS_x0, osrs_t=OVRS_x0, osrs_p=OVRS_x0):
        # Set oversampling variables
        self.osrs_h = osrs_h
        self.osrs_t = osrs_t
        self.osrs_p = osrs_p

        try:
            # Set humidity oversampling
            self.__ctrl_hum(self.osrs_h)
            # Set temperature and pressure oversampling
            self.__ctrl_meas(self.osrs_t, self.osrs_p, self.mode)
        except Exception as e:
            # e = sys.exc_info()[0]
            print("<p>Error: %s</p>" % e)
            return 0

        # Everything went well return 1
        return 1

    #########################################
    # Set internal IIR filter               #
    #########################################
    def set_filter(self, filter_coefficient=filter_0):
        self.filter = filter_coefficient

        try:
            # Write config register with new filter setting
            self.__config(self.t_sb, self.filter, self.spi3w_en)
            return 1
        except Exception as e:
            # print("Error in set_filter()")
            print("<p>Error: %s</p>" % e)
            return 0

    #########################################
    # Set internal SPI interface            #
    #########################################
    def set_spi(self, spi3w_en=SPI_OFF):
        self.spi3w_en = spi3w_en

        try:
            # Write config register with new spi setting
            self.__config(self.t_sb, self.filter, self.spi3w_en)
            return 1
        except Exception as e:
            # print("Error in set_filter()")
            print("<p>Error: %s</p>" % e)
            return 0

    #########################################
    # Reset command                         #
    #########################################
    def write_reset(self):
        register = 0xE0
        data = 0xB6    # Reset command to register, other values than 0xB6 has no effect
        r_len = 0
        delay = 10

        command = ([register, data], r_len, delay)

        try:
            self.read_write(command)
            return 1
        except Exception as e:
            # print("Error in set_filter()")
            print("<p>Error: %s</p>" % e)
            return 0

    #########################################
    # read ID register                      #
    #########################################
    def read_id(self):
        register = 0xD0
        data = 0x00
        r_len = 1
        delay = 10

        command = ([register, data], r_len, delay)

        try:
            return hex(self.read_write(command)[0])
        except Exception as e:
            # print("Error in set_filter()")
            print("<p>Error: %s</p>" % e)
            return 0

    #########################################
    # read STATUS register                  #
    #########################################
    def read_status(self):
        register = 0xF3
        data = 0x00
        r_len = 1
        delay = 10

        command = ([register, data], r_len, delay)

        try:
            self.status = self.read_write(command)[0]
            self.measuring = (self.status >> 3) & 0b1
            self.im_update = self.status & 0b1
            return [self.status, self.measuring, self.im_update]
        except Exception as e:
            # print("Error in set_filter()")
            print("<p>Error: %s</p>" % e)
            return 0

    #########################################
    # read calibration registers            #
    #########################################
    def __read_calib(self):
        # Calibration data registers are 0x88-0xA1(25 bytes) and 0xE1-0xE7(7 bytes) total 32 bytes
        # command format ([Register, Data Bytes], Number of bytes to read, Additional sleep(ms))
        # If Number of bytes to read > 0 then read_write function reads from I2C Bus
        # Otherwise Write operation is performed.
        register1 = 0x88
        register2 = 0xE1
        data = 0x00
        r_len1 = 26
        r_len2 = 7
        delay = 10

        command1 = ([register1, data], r_len1, delay)
        command2 = ([register2, data], r_len2, delay)

        try:
            self.calib[0:26] = self.read_write(command1)
            self.calib[26:32] = self.read_write(command2)
        except Exception as e:
            # print("Error in set_filter()")
            print("<p>Error: %s</p>" % e)
            return 0

        # Assign fetched data to trim parameters
        # Temperature trimming params
        # unsigned are ok, signed must be calculated with twos complement
        bytes_short = 16
        bytes_char = 8

        self.dig_T1 = (self.calib[1] << 8) + (self.calib[0] & 0xffff)
        self.dig_T2 = self.__twos_complement(((self.calib[3] << 8) + self.calib[2]), bytes_short)
        self.dig_T3 = self.__twos_complement((self.calib[5] << 8) + self.calib[4], bytes_short)

        # Pressure trimming params
        self.dig_P1 = ((self.calib[7] << 8)+self.calib[6]) & 0xffff
        self.dig_P2 = self.__twos_complement((self.calib[9] << 8) + self.calib[8], bytes_short)
        self.dig_P3 = self.__twos_complement((self.calib[11] << 8) + self.calib[10], bytes_short)
        self.dig_P4 = self.__twos_complement((self.calib[13] << 8) + self.calib[12], bytes_short)
        self.dig_P5 = self.__twos_complement((self.calib[15] << 8) + self.calib[14], bytes_short)
        self.dig_P6 = self.__twos_complement((self.calib[17] << 8) + self.calib[16], bytes_short)
        self.dig_P7 = self.__twos_complement((self.calib[19] << 8) + self.calib[18], bytes_short)
        self.dig_P8 = self.__twos_complement((self.calib[21] << 8) + self.calib[20], bytes_short)
        self.dig_P9 = self.__twos_complement((self.calib[23] << 8) + self.calib[22], bytes_short)

        # Humidity trimming params
        self.dig_H1 = self.calib[25] & 0xff
        self.dig_H2 = self.__twos_complement((self.calib[27] << 8) + self.calib[26], bytes_short)
        self.dig_H3 = self.calib[28] & 0xff
        self.dig_H4 = self.__twos_complement((self.calib[29] << 4) + (self.calib[30] & 0x0f), bytes_short)
        self.dig_H5 = self.__twos_complement((self.calib[31] << 4) + ((self.calib[30] & 0xf0) >> 4), bytes_short)
        self.dig_H6 = self.__twos_complement(self.calib[32], bytes_char)

        # Everything went well return 1
        return 1

    #########################################
    # read raw signals registers            #
    #########################################
    def read_raw_signals(self):
        # RAW Signals memory area is 0xF7-0xFE so there is 8 bytes
        # command format ([Register, Data Bytes], Number of bytes to read, Additional sleep(ms))
        # If Number of bytes to read > 0 then read_write function reads from I2C Bus
        # Otherwise Write operation is performed.
        register = 0xF7
        data = 0x00
        r_len = 8
        delay = 10

        command = ([register, data], r_len, delay)

        try:
            self.raw_data = self.read_write(command)
            self.raw_pressure = (((self.raw_data[0] << 8) + self.raw_data[1]) << 4) + ((self.raw_data[2] >> 4) & 0x0f)
            self.raw_temperature = ((((self.raw_data[3] << 8) + self.raw_data[4]) << 4)
                                    + ((self.raw_data[5] >> 4) & 0x0f))
            self.raw_humidity = ((self.raw_data[6] << 8) + self.raw_data[7])
        except Exception as e:
            # print("Error in set_filter()")
            print("<p>Error: %s</p>" % e)
            return 0

        # Everything went well return 1
        return 1

    #########################################
    # read compensated signals              #
    #########################################
    def read_compensated_signals(self):
        # Assuming that raw signals are already read
        # Compensation is performed like in datasheet appendix 8
        # 8.1 Compensation formulas in double precision floating point

        try:
            # Get Temperature t_fine value
            t_fine = self.__comp_temperature()
            self.temperature = t_fine / 5120.0  # Output value of “51.23” equals 51.23 DegC
            self.calibrated_temperature = (self.temperature
                                           + (0 if self.calibration_temperature is 0 else self.calibration_temperature))

            # Get pressure
            self.pressure = self.__comp_pressure(t_fine) / 100.0
            self.calibrated_pressure = (self.pressure
                                        + (0 if self.calibration_pressure is 0 else self.calibration_pressure))

            # Get Humidity
            self.humidity = self.__comp_humidity(t_fine)
            self.calibrated_humidity = (self.humidity
                                        + (0 if self.calibration_humidity is 0 else self.calibration_humidity))

            return 1
        except Exception as e:
            # print("Error in set_filter()")
            print("<p>Error: %s</p>" % e)
            return 0

    #########################################
    # Temperature compensation formulas     #
    # returns t_fine, because it is used in #
    # other compensation formulas           #
    #########################################
    def __comp_temperature(self):
        var1 = ((self.raw_temperature / 16384.0) - (self.dig_T1 / 1024.0)) * self.dig_T2
        var2 = ((((self.raw_temperature / 131072.0) - (self.dig_T1 / 8192.0))
                 * ((self.raw_temperature / 131072.0) - (self.dig_T1 / 8192.0))) * self.dig_T3)
        # t_fine = var1 + var2
        return var1 + var2

    #############################################################
    # Pressure compensation formulas                            #
    # Returns pressure in Pa as double.                         #
    # Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa #
    #############################################################
    def __comp_pressure(self, t_fine):
        var1 = (t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * self.dig_P6 / 32768.0
        var2 = var2 + var1 * self.dig_P5 * 2.0
        var2 = (var2 / 4.0) + (self.dig_P4 * 65536.0)
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1
        if var1 == 0.0:
            return 0    # avoid exception caused by division by zero

        p = 1048576.0 - self.raw_pressure
        p = (p - (var2 / 4096.0)) * 6250.0 / var1
        var1 = self.dig_P9 * p * p / 2147483648.0
        var2 = p * self.dig_P8 / 32768.0
        p = p + (var1 + var2 + self.dig_P7) / 16.0
        return p

    ##################################################
    # Humidity compensation formulas                 #
    # Returns humidity in %rH as as double.          #
    # Output value of “46.332” represents 46.332 %rH #
    ##################################################
    def __comp_humidity(self, t_fine):
        var_h = t_fine - 76800.0
        var_h = ((self.raw_humidity - (self.dig_H4 * 64.0 + (self.dig_H5 / 16384.0 * var_h)))
                 * (self.dig_H2 / 65536.0 * (1.0 + self.dig_H6 / 67108864.0 * var_h
                                             * (1.0 + self.dig_H3 / 67108864.0 * var_h))))
        var_h = var_h * (1.0 - self.dig_H1 * var_h / 524288.0)
        if var_h > 100.0:
            var_h = 100.0
        elif var_h < 0.0:
            var_h = 0.0

        return var_h

    #########################################
    # write ctrl_hum register               #
    #########################################
    def __ctrl_hum(self, osrs_h=OVRS_x0):
        register = 0xF2
        r_len = 0
        delay = 10

        # Make byte to send
        # data = ((0x00 << 3) + osrs_h)
        data = osrs_h

        command = ([register, data], r_len, delay)
        self.read_write(command)

    #########################################
    # write ctrl_meas register              #
    #########################################
    def __ctrl_meas(self, osrs_t=OVRS_x0, osrs_p=OVRS_x0, mode=MODE_SLEEP):
        register = 0xF4
        r_len = 0
        delay = 10

        # Make byte to send
        data = ((((osrs_t << 3) + osrs_p) << 2) + mode)

        command = ([register, data], r_len, delay)
        self.read_write(command)

    #########################################
    # write config register                 #
    #########################################
    def __config(self, t_sb, iir_filter, spi3w_en):
        register = 0xF5
        r_len = 0
        delay = 10

        # Make byte to send
        data = ((((t_sb << 3) + iir_filter) << 2) + spi3w_en)

        command = ([register, data], r_len, delay)
        self.read_write(command)

    ##############################################################
    # set_pressure_calibration                                   #
    # level = known level from sea                               #
    # pressure =  current local pressure normalized to sea level #
    ##############################################################
    def set_pressure_calibration(self, level, pressure):
        self.calibration_pressure = (pressure - level / 8) - self.pressure

    #########################################
    # get current altitude                  #
    # pressure = current sea level pressure #
    #########################################
    def get_altitude(self, pressure):
        return (pressure - self.calibrated_pressure) * 8

    ###########################################
    # read and write data to the bme280/(I2C) #
    ###########################################
    def read_write(self, command):
        if command[1] <= 0:
            # Write to I2C-bus
            # write_i2c_block_data(address, offset, [data_bytes])
            self.bus.write_i2c_block_data(self.address, command[0][0], command[0][1:])
            sleep(command[2]/1000)
            return 1
        else:
            # Read from I2C-Bus
            # read_i2c_block_data(address, register, number_of_data_bytes)
            return self.bus.read_i2c_block_data(self.address, command[0][0], command[1])

    #########################################
    # Twos complement calculation           #
    #########################################
    @staticmethod
    def __twos_complement(input_value, num_bits):
        """Calculates a two's complement integer from the given input value's bits."""

        mask = 2 ** (num_bits - 1)
        return -(input_value & mask) + (input_value & (mask-1))
