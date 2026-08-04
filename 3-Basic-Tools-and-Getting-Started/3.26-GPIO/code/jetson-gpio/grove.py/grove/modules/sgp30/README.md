## Using the [Grove SGP30 module](http://wiki.seeedstudio.com/Grove-VOC_and_eCO2_Gas_Sensor-SGP30/) with the [GrovePi Plus](http://wiki.seeedstudio.com/GrovePi_Plus/)
Tested with RaspberryPi 3B v1.2

### Setting It Up
Assuming that [grove.py](https://github.com/Seeed-Studio/grove.py) is already installed.
just import the class SGP30 from grove.modules.sgp30

### Running it
There is example in example folder.
```bash
sudo python3 sgp30_example.py
```

To run the script, you need to have [Grove SGP30 Module](http://wiki.seeedstudio.com/Grove-VOC_and_eCO2_Gas_Sensor-SGP30/) installed to the one of the I2C ports of the GrovePi Plus.

The output of this example should looks like this:
```bash
On-chip self-test succesfull!
0xd400
Product type:  0000
Product version:  00100000, 32
Get serial succesfull!
Serial part 1:  0000000000000000
Serial part 2:  0000000010011010
Serial part 3:  0010001111011100
SGP30 Init Ok!

CO2eq: 400 ppm
TVOC: 0 ppb
Measurement cycle length: 13.823694ms
CO2eq baseline: 0, TVOC baseline: 0
Total cycle length: 25.894204ms
```

**Note! Sensor needs little time to initialize so measurement won't work instantly!!**