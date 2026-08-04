#!/usr/bin/env python
#
# Library for Grove - CO2 & Temperature & Humidity Sensor for Arduino (SCD30) - 3-in-1
# (https://www.seeedstudio.com/Grove-CO2-Temperature-Humidity-Sensor-SCD30-p-2911.html)
#

'''
## License

The MIT License (MIT)

Copyright (C) 2022  matsujirushi

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

from typing import NoReturn
from grove.i2c import Bus
import struct
import time


class GroveCo2Scd30(object):
    __COMMAND_TRIGGER_CONTINUOUS_MEASUREMENT = 0x0010
    __COMMAND_STOP_CONTINUOUS_MEASUREMENT = 0x0104
    __COMMAND_SET_MEASUREMENT_INTERVAL = 0x4600
    __COMMAND_GET_DATA_READY_STATUS = 0x0202
    __COMMAND_READ_MEASUREMENT = 0x0300
    __COMMAND_ACTIVATE_ASC = 0x5306
    __COMMAND_SET_FRC = 0x5204
    __COMMAND_SET_TEMPRATURE_OFFSET = 0x5403
    __COMMAND_ALTITUDE_COMPENSATION = 0x5102
    __COMMAND_READ_FIRMWARE_VERSION = 0xd100
    __COMMAND_SOFT_RESET = 0xd304

    def __init__(self, address=0x61, bus=1):
        self.address = address
        self.bus = Bus(bus)

        self.set_measurement_interval(2)
        self.trigger_continuous_measurement()

    @staticmethod
    def _calc_crc(data: list) -> int:
        crc = 0xff

        for d in data:
            crc ^= d

            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x31) & 0xff
                else:
                    crc = (crc << 1) & 0xff

        return crc

    def _write(self, cmd: int, data: list):
        write_data = list(struct.pack(">H", cmd))
        if data is not None:
            for d in data:
                write_data.extend(struct.pack(">H", d))
                write_data.append(GroveCo2Scd30._calc_crc(struct.pack(">H", d)))

        write_msg = self.bus.msg.write(self.address, write_data)
        self.bus.i2c_rdwr(write_msg)

    def _read(self, address: int, data_number: int) -> list:
        write_data = list(struct.pack(">H", address))

        write_msg = self.bus.msg.write(self.address, write_data)
        self.bus.i2c_rdwr(write_msg)

        read_msg = self.bus.msg.read(self.address, 3 * data_number)
        self.bus.i2c_rdwr(read_msg)

        result = []
        for i in range(data_number):
            d = read_msg.buf[i*3:i*3+2]
            if GroveCo2Scd30._calc_crc(d) != read_msg.buf[i*3+2][0]:
                raise ValueError("CRC mismatch")

            result.append(struct.unpack(">H", d)[0])

        return result

    def trigger_continuous_measurement(self, pressure: int = 0):
        self._write(self.__COMMAND_TRIGGER_CONTINUOUS_MEASUREMENT, [pressure])

    def stop_continuous_measurement(self):
        self._write(self.__COMMAND_STOP_CONTINUOUS_MEASUREMENT, None)

    def set_measurement_interval(self, interval: int):
        self._write(self.__COMMAND_SET_MEASUREMENT_INTERVAL, [interval])

    def get_measurement_interval(self) -> int:
        data = self._read(self.__COMMAND_SET_MEASUREMENT_INTERVAL, 1)

        return data[0]

    def get_data_ready_status(self) -> bool:
        data = self._read(self.__COMMAND_GET_DATA_READY_STATUS, 1)

        return True if data[0] == 1 else False

    def read_measurement(self) -> tuple:
        data = self._read(self.__COMMAND_READ_MEASUREMENT, 6)

        data_bytes = struct.pack(">HHHHHH", data[0], data[1], data[2], data[3], data[4], data[5])
        data_floats = struct.unpack(">fff", data_bytes)
        co2 = data_floats[0]
        temp = data_floats[1]
        humi = data_floats[2]

        return co2, temp, humi

    def set_forced_recalibration(self, co2: float):
        self._write(self.__COMMAND_SET_FRC, [int(co2)])

    def set_automatic_self_calibration(self, activate: bool):
        self._write(self.__COMMAND_ACTIVATE_ASC, [1 if activate else 0])

    def get_automatic_self_calibration(self) -> bool:
        data = self._read(self.__COMMAND_ACTIVATE_ASC, 1)

        return True if data[0] == 1 else False

    def set_temperature_offset(self, offset: float):
        self._write(self.__COMMAND_SET_TEMPRATURE_OFFSET, [int(offset * 100)])

    def get_temperature_offset(self) -> float:
        data = self._read(self.__COMMAND_SET_TEMPRATURE_OFFSET, 1)

        return float(data[0]) / 100

    def set_altitude_compensation(self, altitude: int):
        self._write(self.__COMMAND_ALTITUDE_COMPENSATION, [altitude])

    def get_altitude_compensation(self) -> int:
        data = self._read(self.__COMMAND_ALTITUDE_COMPENSATION, 1)

        return data[0]

    def read(self) -> tuple:
        if not self.get_data_ready_status():
            return None

        return self.read_measurement()


def main() -> NoReturn:
    sensor = GroveCo2Scd30()

    while True:
        if sensor.get_data_ready_status():
            co2, temperature, humidity = sensor.read()
            print(f"CO2 concentration is {co2:.1f} ppm")
            print(f"Temperature in Celsius is {temperature:.2f} C")
            print(f"Relative Humidity is {humidity:.2f} %")

        time.sleep(1)


if __name__ == "__main__":
    main()
