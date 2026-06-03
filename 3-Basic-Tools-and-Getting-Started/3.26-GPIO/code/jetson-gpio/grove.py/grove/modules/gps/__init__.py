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

import serial
from time import sleep
import datetime


class GPS:
    def __init__(self, port='/dev/ttyAMA0', baud=9600, timeout=1):
        # Initialize the serial line
        # Serial line timeout has to be set because use of the readline()-function, otherwise can block forever
        self.serial = serial.Serial(port, baud, timeout=timeout)
        self.serial.flush()
        self.line = []

        # Define variables
        # Derived variables from message variables
        # gps time and date to Python datetime
        self.utc_datetime = datetime.datetime.now()
        # Full location string as Degrees and Decimal Minutes(Most commonly used format)
        self.location_DDM = ""

        # __GGA message has
        self.GGA_utc_time = self.utc_datetime.strftime("%H%M%S.%f")
        self.GGA_latitude = ""
        self.GGA_NS_indicator = ""
        self.GGA_longitude = ""
        self.GGA_EW_indicator = ""
        self.GGA_position_fix_indicator = ""
        self.GGA_fix_indicator = {0: "Fix not available or invalid",
                                  1: "gps SPS Mode, fix valid",
                                  2: "Differential gps, SPS Mode, fix valid",
                                  3: "Not supported",
                                  4: "Not supported",
                                  5: "Not supported",
                                  6: "Dead Reckoning Mode, fix valid"}
        self.GGA_satellites_used = ""
        self.GGA_HDOP = ""
        self.GGA_MSL_altitude = ""
        self.GGA_units1 = ""
        self.GGA_geoid_separation = ""
        self.GGA_units2 = ""
        self.GGA_age_of_diff_corr = ""
        self.GGA_diff_ref_stat_id = ""

        # __GSA message has
        self.GSA_mode1 = ""
        self.GSA_mode2 = ""
        self.GSA_satellite_used = ["", "", "", ""] * 12
        self.GSA_PDOP = ""
        self.GSA_HDOP = ""
        self.GSA_VDOP = ""

        # __GSV message has
        self.GSV_number_of_messages = 0
        self.GSV_message_number = 0
        self.GSV_satellites_in_view = 0
        self.GSV_satellite_data = [[""]]  # Size modified in __GSV base

        # __RMC message has
        self.RMC_utc_time = self.utc_datetime.strftime("%H%M%S.%f")
        self.RMC_status = ""
        self.RMC_latitude = ""  # same as __GGA
        self.RMC_NS_indicator = ""  # same as __GGA
        self.RMC_longitude = ""  # same as __GGA
        self.RMC_EW_indicator = ""  # same as __GGA
        self.RMC_speed_over_ground = ""
        self.RMC_course_over_ground = ""
        self.RMC_date = self.utc_datetime.strftime("%d%m%y")
        self.RMC_magnetic_variation = ""
        self.RMC_mv_EW_indicator = ""
        self.RMC_mode = ""

    # Check if serial interface buffer has unread bytes
    def new_data(self):
        if self.serial.in_waiting:
            return 1
        else:
            return 0

    def read(self):
        try:
            # If there is something to read then proceed
            if self.new_data():
                # Read one line of data
                if self.__read_line(3):
                    # Get the type without the sender to support all constellations
                    message_type = self.line[0][-3:]
                    # Select correct function to proceed if read successful
                    if message_type == "GGA":
                        return self.__GGA(self.line)
                    elif message_type == "GSA":
                        return self.__GSA(self.line)
                    elif message_type == "GSV":
                        return self.__GSV(self.line)
                    elif message_type == "RMC":
                        return self.__RMC(self.line)
        except IndexError:
            pass

        return 0

    def __read_line(self, tries):
        """
        Attempts n times at most to get valid data from gps
        Returns as soon as valid data is found
        If data read fails, then return empty list to self.line variable
        """
        for _ in range(tries):
            try:
                # Read one line from serial
                # Decode it to utf-8-format
                line = self.serial.readline().decode('utf-8')
                # Add comma before checksum *
                index = line.find("*")
                line = line[:index] + "," + line[index:]

                # Strip leading and trailing spaces and
                # Convert comma separated string to list with split
                self.line = line.strip().split(",")
                return 1
            except Exception:
                pass

        # Should never reach here if everything is ok
        self.line = []
        return 0

    #############################################
    # Message ID __GGA ($GPGGA)                 #
    # Global Positioning System Fixed Data      #
    #############################################
    def __GGA(self, line):
        if self.__check_checksum(line):
            self.GGA_utc_time = line[1]
            self.GGA_latitude = line[2]
            self.GGA_NS_indicator = line[3]
            self.GGA_longitude = line[4]
            self.GGA_EW_indicator = line[5]
            self.GGA_position_fix_indicator = line[6]
            self.GGA_satellites_used = line[7]
            self.GGA_HDOP = line[8]
            self.GGA_MSL_altitude = line[9]
            self.GGA_units1 = line[10]
            self.GGA_geoid_separation = line[11]
            self.GGA_units2 = line[12]
            self.GGA_age_of_diff_corr = line[13]
            self.GGA_diff_ref_stat_id = line[14]

            self.__update_datetime()
            self.__update_location("__GGA")
            return 1
        else:
            # print("Checksum failed!")
            return 0

    #############################################
    # Message ID __GSA ($GPGSA)                 #
    # GNSS DOP and Active Satellites            #
    #############################################
    def __GSA(self, line):
        if self.__check_checksum(line):
            # print("GSA Checksum OK!")
            self.GSA_mode1 = line[1]
            self.GSA_mode2 = line[2]
            for i in range(0, 12):
                self.GSA_satellite_used[i] = line[i+3]
            self.GSA_PDOP = line[16]
            self.GSA_HDOP = line[17]
            self.GSA_VDOP = line[18]

            return 1
        else:
            # print("GSA Checksum failed!")
            return 0

    #############################################
    # Message ID __GSV ($GPGSV)                 #
    # GNSS Satellites in View                   #
    #############################################
    def __GSV(self, line):
        if self.__check_checksum(line):
            self.GSV_number_of_messages = int(line[1])
            self.GSV_message_number = int(line[2])
            self.GSV_satellites_in_view = int(line[3])

            # Modify satellite data variable size depend on number of reported satellites in messages
            # Only when the first message of the message bunch has arrived
            if self.GSV_message_number == 1:
                self.GSV_satellite_data = [["", "", "", ""]]*self.GSV_number_of_messages*4

            range_end = round((len(line)-5)/4)
            for i in range(0, range_end):
                self.GSV_satellite_data[i + 4*(self.GSV_message_number-1)] = [line[4+i*4],
                                                                              line[5+i*4],
                                                                              line[6+i*4],
                                                                              line[7+i*4]]
            return 1
        else:
            return 0

    #############################################
    # Message ID __RMC ($GPRMC)                 #
    # Recommended Minimum Specific GNSS Data    #
    #############################################
    def __RMC(self, line):
        if self.__check_checksum(line):
            self.RMC_utc_time = line[1]
            self.RMC_status = line[2]
            self.RMC_latitude = line[3]
            self.RMC_NS_indicator = line[4]
            self.RMC_longitude = line[5]
            self.RMC_EW_indicator = line[6]
            self.RMC_speed_over_ground = line[7]
            self.RMC_course_over_ground = line[8]
            self.RMC_date = line[9]
            self.RMC_magnetic_variation = line[10]
            self.RMC_mv_EW_indicator = line[11]
            self.RMC_mode = line[12]

            self.__update_datetime()
            self.__update_location("__RMC")
            return 1
        else:
            return 0

    # get Python datetime format from message strings
    def __update_datetime(self):
        # hhmmss.ss
        # which received time is newer
        time = self.GGA_utc_time if self.GGA_utc_time > self.RMC_utc_time else self.RMC_utc_time
        self.utc_datetime = datetime.datetime.strptime(self.RMC_date + time, "%d%m%y%H%M%S.%f")

    # self.position_DDM
    def __update_location(self, message):

        if message == "__GGA":
            self.location_DDM = (self.GGA_latitude[:2] + chr(176) +
                                 self.GGA_latitude[2:] + "'" +
                                 self.GGA_NS_indicator +
                                 self.GGA_longitude[:2] + chr(176) +
                                 self.GGA_longitude[2:] + "'" +
                                 self.GGA_EW_indicator)
        elif message == "__RMC":
            self.location_DDM = (self.RMC_latitude[:2] + chr(176) +
                                 self.RMC_latitude[2:] + "'" +
                                 self.RMC_NS_indicator +
                                 self.RMC_longitude[:2] + chr(176) +
                                 self.RMC_longitude[2:] + "'" +
                                 self.RMC_EW_indicator)

        if len(self.location_DDM) <= 4:
            self.location_DDM = "Unavailable"

    ########################################################################################
    # Like name says, calculates message checksum and compares it to the messages checksum #
    ########################################################################################
    @staticmethod
    def __check_checksum(line):
        checksum = 0x0

        stringline = line[0]
        for i in range(1, len(line)):
            # Do not add comma before *checksum
            if i is not len(line)-1:
                stringline += ","
            stringline += line[i]

        for char in stringline:
            if char == "$":
                # Pass the starting character
                pass
            elif char == "*":
                # Stop XOR'in when '*' found
                break
            else:
                # XOR character to checksum
                checksum ^= ord(char)

        # Modify checksum to match the NMEA format
        checksum = ("*" + str(hex(checksum))[2:]).upper()
        if len(checksum) < 3:
            # Add missing 0 to checksum string
            checksum = checksum[:1] + "0" + checksum[1:]

        # Return 1 if checksum matches the line checksum
        if checksum == line[len(line)-1]:
            return 1
        else:
            return 0
