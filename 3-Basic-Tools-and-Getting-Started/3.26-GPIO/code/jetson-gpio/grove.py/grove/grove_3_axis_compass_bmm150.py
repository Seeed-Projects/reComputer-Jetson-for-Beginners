#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# The MIT License (MIT)
#
# Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
# Copyright (C) 2019  Seeed Technology Co.,Ltd.
#
# Author: Jon Trulson <jtrulson@ics.com>
# Copyright (c) 2016-2017 Intel Corporation.
'''
This code is for
    - `Grove - 3-Axis Digital Compass V2 <https://www.seeedstudio.com/s/Grove-3-Axis-Digital-Compass-V2-p-3034.html>`_

'''

from __future__ import print_function
import time
import bmm150
import math


def main():
    device = bmm150.BMM150()  # Bus number will default to 1
    while (1):
        x, y, z = device.read_mag_data()

        heading_rads = math.atan2(x, y)

        heading_degrees = math.degrees(heading_rads)

        print("Magnetometer x: {0:.2f}".format(x), end=' ')
        print(" y: {0:.2f}".format(y), end=' ')
        print(" z: {0:.2f}".format(z), end=' ')
        print(" uT")

        print('heading(axis_Y point to): {0:.2f} degree'.format(heading_degrees))
        time.sleep(.250)

if __name__ == '__main__':
    main()