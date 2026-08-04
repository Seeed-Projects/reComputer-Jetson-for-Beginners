#!/usr/bin/env python
# -*- coding: utf-8 -*-

# The MIT License (MIT)
#
# Copyright (c) 2020 Ville Laine, Aikuiskoulutus Taitaja
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from time import sleep
from grove.modules.bme280 import bme280

bme_sensor = bme280()

# Set oversampling
# bme280 class defines OVRS_x0, .._x1, .._x2, .._x4, .._x8, .._x16
# set_oversampling(osrs_h(humidity), osrs_t(temperature), osrs_p(pressure))
# Set_oversampling > OVRS_x0 to enable the measurement, OVRS_x0 disables the measurement
bme_sensor.set_oversampling(bme280.OVRS_x16, bme280.OVRS_x16, bme280.OVRS_x16)

# Set internal IIR filter coefficient. 0 = no filter
iir_filter = bme280.filter_16
bme_sensor.set_filter(iir_filter)

# Know values for pressure correction
current_level_from_sea = 103            # Know height from sealevel m
current_sea_level_pressure = 1027.5     # Forecast data: current pressure at sealevel
count = 0                               # Just for counting delays
calibration_set = 0                     # Help bit

while True:
    try:
        # set mode to FORCE that is one time measurement
        # bme280.MODE_SLEEP, ...FORCE, ...NORMAL
        # If normal mode also set t_sb that is standby time between measurements
        # if not specified is set to 1000ms bme280.t_sb_1000
        # Returns 1 on success 0 otherwise
        if not bme_sensor.set_mode(bme280.MODE_FORCE):
            print("\nMode change failed!")

        # Measure raw signals measurements are put in bme280.raw_* variables
        # Returns 1 on success otherwise 0
        if not bme_sensor.read_raw_signals():
            print("\nError in measurement!")

        # Compensate the raw signals
        if not bme_sensor.read_compensated_signals():
            print("\nError compensating values")

        # Set pressure calibration after couple measurements
        # Sensor need to settle after soft reset given in the beginning
        # Delay depends on IIR filter step response and the 90% response is 2**iir_filter*2 seconds
        response_time = (2**iir_filter)*2
        if count == response_time:
            bme_sensor.set_pressure_calibration(level=current_level_from_sea, pressure=current_sea_level_pressure)
            count = response_time + 1
            calibration_set = 1
            # Update the compensated values because new calibration value is given
            if not bme_sensor.read_compensated_signals():
                print("\nError compensating values")
            print("Sensor compensation is set")
        elif count < response_time:
            print("Wait for sensor to settle before setting compensation!", response_time-count, "s")
            count += 1

        # Only print out signals after calibration
        if calibration_set:
            # Only works if pressure calibration is done with set_pressure_calibration()
            altitude = bme_sensor.get_altitude(current_sea_level_pressure)

            # Print out the data
            print("Temperature: %.2f" % bme_sensor.temperature, chr(176) + "C")
            print("Pressure: %.2fhPa, where correction is %.2fhPa, sensor reading is %.2fhPa"
                  % (bme_sensor.calibrated_pressure, bme_sensor.calibration_pressure, bme_sensor.pressure))
            print("Humidity: %.2f" % bme_sensor.humidity, "%RH")
            print("altitude from sea level: %.3fm, %.3f" % (altitude, bme_sensor.calibrated_pressure + altitude/8))
            print("\n")

        # Sleep 1 second
        sleep(1)
    except KeyboardInterrupt:
        break
