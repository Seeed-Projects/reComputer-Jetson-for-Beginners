#!/usr/bin/env python
#
# This code is for Grove -  Grove - Temperature, Humidity, Pressure and Gas Sensor (BME680)
# (https://www.seeedstudio.com/Grove-Temperature-Humidity-Pressure-and-Gas-Sensor-BME68-p-3109.html)
# which is a multiple function sensor which can measure temperature, pressure, humidity and gas
# at the same time. 
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
import bme680 as bm

class GroveBME680(object):
    def __init__(self):
        snr = bm.BME680(bm.I2C_ADDR_PRIMARY)

        # over sample, make data more accuracy
        snr.set_humidity_oversample(bm.OS_2X)
        snr.set_pressure_oversample(bm.OS_4X)
        snr.set_temperature_oversample(bm.OS_8X)
        snr.set_filter(bm.FILTER_SIZE_3)
        snr.set_gas_status(bm.ENABLE_GAS_MEAS)

        # profile # 0
        snr.set_gas_heater_temperature(320)
        snr.set_gas_heater_duration(150)
        snr.select_gas_heater_profile(0)

        # profile # 1
        # snr.set_gas_heater_profile(200, 150, nb_profile=1)
        # snr.select_gas_heater_profile(1)

        # Initial reading
        snr.get_sensor_data()
        snr.get_sensor_data()

        self.snr = snr

    def read(self):
        if self.snr.get_sensor_data():
            return self.snr.data
        return None


def main():
    import time
    print(\
""" Make sure Grove-Temperature-Humidity-Press-Gas-Sensor(BME680)
   inserted in one I2C slot of Grove-Base-Hat
""")

    sensor = GroveBME680()

    print("Temperature, Pressure, Humidity, Gas")
    while True:
        d = sensor.read()
        if d:
            fmt = '{0:.2f} C, {1:.2f} hPa, {2:.2f} %RH'.format(
                d.temperature, d.pressure, d.humidity)
            if d.heat_stable:
                fmt += ' {1} Ohms'.format(fmt, d.gas_resistance)
            print(fmt)
        time.sleep(1)

if __name__ == '__main__':
	main()
