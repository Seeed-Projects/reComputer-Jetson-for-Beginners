## Using the [Grove BME280 Module](http://wiki.seeedstudio.com/Grove-Barometer_Sensor-BME280/) with the [GrovePi Plus](http://wiki.seeedstudio.com/GrovePi_Plus/)

### Setting It Up
Assuming that [grove.py](https://github.com/Seeed-Studio/grove.py) is already installed.
just import the class bme280 from grove.modules.bme280

### Running it
There is example in example folder.
```bash
sudo python3 bme280_example.py
```

To run the script, you need to have [Grove BME280 Module](http://wiki.seeedstudio.com/Grove-Barometer_Sensor-BME280/) installed to the one of the I2C ports of the GrovePi Plus.

The output of this example should looks like this:
```bash
Wait for sensor to settle before setting compensation! 2 s
Wait for sensor to settle before setting compensation! 1 s
Sensor compensation is set
Temperature: 22.30 °C
Pressure: 1014.62hPa, where correction is 45.62hPa, sensor reading is 969.00hPa
Humidity: 32.44 %RH
altitude from sea level: 103.000m, 1027.500


Temperature: 22.30 °C
Pressure: 1014.63hPa, where correction is 45.62hPa, sensor reading is 969.01hPa
Humidity: 32.44 %RH
altitude from sea level: 102.941m, 1027.500
```