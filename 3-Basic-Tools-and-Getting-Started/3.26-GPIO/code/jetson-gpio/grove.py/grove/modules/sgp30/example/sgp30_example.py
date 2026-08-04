"""
The sgp30 uses a dynamic baseline compensation algorithm and on-chip calibration parameters to provide two
complementary air quality signals. Based on the sensor signals a total VOC signal (TVOC) and a CO2 equivalent signal
(CO2eq) are calculated. Sending an “Init_air_quality” command starts the air quality measurement. After the
“Init_air_quality” command, a “Measure_air_quality” command has to be sent in regular intervals of 1s to ensure proper
operation of the dynamic baseline compensation algorithm.
"""
import time
from grove.modules.sgp30 import SGP30

# Create sensor instance
SGP30_1 = SGP30()

# Test sensor with measure_test command
print(hex(SGP30_1.measure_test()))

# Get feature set version
SGP30_1.get_feature_set_version()

# Get 48bit unique serial ID
SGP30_1.get_serial_id()

# Initialize sensor and set compensation baseline values from file
SGP30_1.init_air_quality()

# measure_air_quality command has to be sent regular intervals 1s to ensure proper operation of
# dynamic baseline correction algorithm
while 1:
    start_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME)

    SGP30_1.measure_air_quality()
    print("\nCO2eq: " + str(SGP30_1.CO2eq) + " ppm\nTVOC: " + str(SGP30_1.TVOC) + " ppb")

    end_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    print("Measurement cycle length: " + str((end_time - start_time)/1000000) + "ms")

    # Get compensation baseline values and display them
    # Store them somewhere to
    SGP30_1.get_baseline()
    print("CO2eq baseline: " + str(SGP30_1.CO2eq_baseline) + ", TVOC baseline: " + str(SGP30_1.TVOC_baseline))


    end_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    print("Total cycle length: " + str((end_time - start_time)/1000000) + "ms")
    # Sleep so that next cycle will be after 1s
    time.sleep(1-((end_time - start_time)/1000000000))
