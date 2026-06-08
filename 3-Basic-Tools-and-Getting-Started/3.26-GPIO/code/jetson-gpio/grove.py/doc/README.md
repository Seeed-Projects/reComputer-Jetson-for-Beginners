

## CLI (command line interface)
After installing `grove.py`, A few CLI commands with prefix `grove_` is available, 
such as `grove_led`, `grove_button`, `grove_ultrasonic_ranger` and etc.  
For I2C Grove devices, the default bus is used (I2C 1 on Pi).  
For digital input & output Grove devices, pin numbers should be provided as the arguments of these commands.  
For analog devices, slot number(n of PCB sink An) should be provided.  

#### Command Reference Table
Some devices need root permission signed in sudo column, the coresponding command must prefix with "sudo" if run with a none-root user.

**Most of the commands will list the available argument/pin when it need argument but not specifying.**

<div>
  <table border="0">
    <tr align="center">
      <th>Grove Devices</th>
      <th>Command</th>
      <th>RPi</th>
      <th>Argument/Comment</th>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-1-Wire-Thermocouple-Amplifier-MAX31850-p-3159.html">1-Wire Thermocouple Amplifier (MAX31850K)</a></td>
      <td>grove_1wire_thermocouple_amplifier_max31850</td>
      <td>&bull;</td>
      <td>1-Wire</td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-3-Axis-Digital-Compass-V2-p-3034.html">3-Axis Digital Compass V2</a></td>
      <td>grove_3_axis_compass_bmm150</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-3-Axis-Digital-Accelerometer-LIS3DHTR-p-4533.html">3-Axis Digital Accelerometer(+/-400g)</a></td>
      <td>grove_3_axis_digital_accelerometer</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-4-Digit-Display-p-1198.html">4 Digit Display</a></td>
      <td>grove_4_digit_display</td>
      <td>y</td>
      <td>arg1 - digital pin</td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-6-Axis-Accelerometer-Gyroscope-BMI08-p-3188.html">6-Axis Accelerometer&Gyroscope</a></td>
      <td>grove_6_axis_accel_gyro_bmi088</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-12-Key-Capacitive-I2C-Touch-Sensor-V2-MPR12-p-3141.html">12 Key Capacitive I2C Touch Sensor V2(MPR121)</a></td>
      <td>grove_12_key_cap_i2c_touch_mpr121</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-16-x-2-LCD-Black-on-Red-p-3197.html">16 x 2 LCD (Black on Red)</a><br>
        <a href="https://www.seeedstudio.com/Grove-16-x-2-LCD-Black-on-Yellow-p-3198.html">16 x 2 LCD (Black on Yellow)</a><br>
        <a href="https://www.seeedstudio.com/Grove-16-x-2-LCD-White-on-Blue-p-3196.html">16 x 2 LCD (White on Blue)</a>
      </td>
      <td>grove_16x2_lcd</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-Air-quality-sensor-v1.3-p-2439.html">Air quality sensor v1.3</a></td>
      <td>grove_air_quality_sensor_v1_3</td>
      <td>y</td>
      <td>arg1 - analog pin</td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-Button-p-766.html">Button</a></td>
      <td>grove_button</td>
      <td>y</td>
      <td>arg1 - digital pin</td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-Capacitive-Touch-Slider-Sensor-CY8C4014LQ-p-3183.html">Capacitive Touch Slide Sensor(CY8C4014LQI)</a></td>
      <td>grove_cap_touch_slider_cy8c</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-Collision-Sensor-p-1132.html">Collision Sensor</a></td>
      <td>grove_collision_sensor</td>
      <td>y</td>
      <td>arg1 - digital pin</td>
    </tr>
    <tr align="center">
      <td><a href="http://www.seeedstudio.com/Grove-Gesture-p-2463.html">Gesture Sensor v1.0</a></td>
      <td>grove_gesture_sensor</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-I2C-High-Accuracy-Temperature-Sensor-MCP980-p-3108.html">I2C High Accuracy Temperature Sensor(MCP9808)</a></td>
      <td>grove_high_accuracy_temperature</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-I2C-Color-Sensor-V2-p-2890.html">I2C Color Sensor V2</a>
      </td>
      <td>grove_i2c_color_sensor_v2</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="http://www.seeedstudio.com/Grove-I2C-Motor-Driver-p-907.html">I2C Motor Driver</a>
      </td>
      <td>grove_i2c_motor_driver</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-I2C-Thermocouple-Amplifier-MCP960-p-3199.html">I2C Thermocouple Amplifier (MCP9600)</a>
      </td>
      <td>grove_i2c_thermocouple_amplifier_mcp9600</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-IMU-9DOF-ICM20600-AK0991-p-3157.html">IMU 9DOF (ICM20600+AK09918)</a>
      </td>
      <td>grove_imu_9dof_icm20600_ak09918</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-OLED-Display-1-1-p-824.html">OLED Display 1.12"</a><br>
        <a href="https://www.seeedstudio.com/Grove-OLED-Display-1-12-V2-p-3031.html">OLED Display 1.12" V2</a><br>
      </td>
      <td>grove_lcd_1.2inches</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Red-LED-p-1142.html">Red LED</a><br>
        <a href="https://www.seeedstudio.com/Grove-Green-LED-p-1144.html">Green LED</a><br>
        <a href="https://www.seeedstudio.com/Grove-Purple-LED-3m-p-1143.html">Purple LED</a><br>
        <a href="https://www.seeedstudio.com/Grove-White-LED-p-1140.html">White LED</a>
      </td>
      <td>grove_led</td>
      <td>y</td>
      <td>arg1 - digital pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Light-Sensor-v1.2-p-2727.html">Light Sensor V1.2</a>
      </td>
      <td>grove_light_sensor_v1_2</td>
      <td>y</td>
      <td>arg1 - analog pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Loudness-Sensor-p-1382.html">Loudness Sensor</a>
      </td>
      <td>grove_loudness_sensor</td>
      <td>y</td>
      <td>arg1 - analog pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Mech-Keycap-p-3138.html">Mech Keycap</a>
      </td>
      <td>grove_mech_keycap</td>
      <td>&bull;</td>
      <td>arg1 - PWM pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-mini-PIR-motion-sensor-p-2930.html">mini PIR motion sensor</a><br>
        <a href="https://www.seeedstudio.com/Grove-PIR-Motion-Sensor-p-802.html">PIR Motion Sensor</a>
      </td>
      <td>grove_mini_pir_motion_sensor</td>
      <td>y</td>
      <td>arg1 - digital pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Moisture-Sensor-p-955.html">Moisture Sensor</a>
      </td>
      <td>grove_moisture_sensor</td>
      <td>y</td>
      <td>arg1 - analog pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-5-Way-Switch-p-3136.html">5-Way Switch</a><br>
        <a href="https://www.seeedstudio.com/Grove-6-Position-DIP-Switch-p-3137.html">6-Position DIP Switch</a>
      </td>
      <td>grove_multi_switch<br>
          <b>or</b><br>
          grove_multi_switch_poll
      </td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-OLED-Display-0.96%22-p-781.html">OLED Display 0.96"</a>
      </td>
      <td>grove_oled_display_128x64</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Optical-Rotary-Encoder-TCUT1600X0-p-3142.html">Optical Rotary Encoder(TCUT1600X01)</a>
      </td>
      <td>grove_optical_rotary_encoder</td>
      <td>y</td>
      <td>arg1 - digital pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Piezo-Vibration-Sensor-p-1411.html">Piezo Vibration Sensor</a>
      </td>
      <td>grove_piezo_vibration_sensor</td>
      <td>y</td>
      <td>arg1 - digital pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Buzzer-p-768.html">Buzzer</a>
      </td>
      <td>grove_pwm_buzzer</td>
      <td>&bull;</td>
      <td>PWM</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Recorder-v3.0-p-2709.html">Recorder v3.0</a>
      </td>
      <td>grove_recorder_v3_0</td>
      <td>y</td>
      <td>
<pre>
arg1 - digital pin
arg2 - record duration
       in seconds
</pre>
      </td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Relay-p-769.html">Relay</a>
      </td>
      <td>grove_relay</td>
      <td>y</td>
      <td>arg1 - digital pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Rotary-Angle-Sensor(P)-p-1242.html">Rotary Angle Sensor(P)</a>
      </td>
      <td>grove_rotary_angle_sensor</td>
      <td>y</td>
      <td>arg1 - analog pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Round-Force-Sensor-FSR40-p-3110.html">Round Force Sensor FSR402</a>
      </td>
      <td>grove_round_force_sensor</td>
      <td>y</td>
      <td>arg1 - analog pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Red-LED-Button-p-3096.html">Red LED Button</a><br>
        <a href="https://www.seeedstudio.com/Grove-Yellow-LED-Button-p-3101.html">Yellow LED Button</a><br>
        <a href="https://www.seeedstudio.com/Grove-Blue-LED-Button-p-3104.html">Blue LED Button</a>
      </td>
      <td>grove_ryb_led_button</td>
      <td>y</td>
      <td>arg1 - digital pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Servo-p-1241.html">Servo</a>
      </td>
      <td>grove_servo</td>
      <td>y</td>
      <td>arg1 - digital pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Slide-Potentiometer-p-1196.html">Slide Potentiometer</a>
      </td>
      <td>grove_slide_potentiometer</td>
      <td>y</td>
      <td>arg1 - analog pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Sound-Sensor-p-752.html">Sound Sensor</a>
      </td>
      <td>grove_sound_sensor</td>
      <td>y</td>
      <td>arg1 - analog pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Step-Counter-BMA45-p-3189.html">Step Counter(BMA456)</a>
      </td>
      <td>grove_step_counter_bma456</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Switch%28P%29-p-1252.html">Switch(P)</a>
      </td>
      <td>grove_switch</td>
      <td>y</td>
      <td>arg1 - digital pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Temperature-Humidity-Pressure-and-Gas-Sensor-BME68-p-3109.html">Temperature, Humidity, Pressure and Gas Sensor (BME680)</a>
      </td>
      <td>grove_temperature_humidity_bme680</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Temperature-Humidity-Sensor-DHT1-p-745.html">Temperature & Humidity Sensor (DHT11)</a><br>
        <a href="https://www.seeedstudio.com/Grove-Temperature-Humidity-Sensor-Pro-AM230-p-838.html">Temperature & Humidity Sensor Pro (AM2302)</a>
      </td>
      <td>grove_temperature_humidity_sensor</td>
      <td>y</td>
      <td>
<pre>
arg1 - digital pin
arg2 - dht_type, 
       could be 11 or 22,
       represent DHT11, DHT22/AM2302
</pre>
      </td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Temperature-Humidity-Sensor-SHT3-p-2655.html">Temperature & Humidity Sensor (SHT31)</a>
      </td>
      <td>grove_temperature_humidity_sht31</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Temperature-Sensor-p-774.html">Temperature Sensor</a>
      </td>
      <td>grove_temperature_sensor</td>
      <td>y</td>
      <td>arg1 - analog pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Thumb-Joystick-p-935.html">Thumb Joystick</a>
      </td>
      <td>grove_thumb_joystick</td>
      <td>y</td>
      <td>arg1 - analog pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Tilt-Switch-p-771.html">Tilt Switch</a>
      </td>
      <td>grove_tilt_switch</td>
      <td>y</td>
      <td>arg1 - digital pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Time-of-Flight-Distance-Sensor-VL53L0-p-3086.html">Time of Flight Distance Sensor VL53L0X</a>
      </td>
      <td>grove_time_of_flight_distance</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Touch-Sensor-p-747.html">Touch Sensor</a>
      </td>
      <td>grove_touch_sensor</td>
      <td>y</td>
      <td>arg1 - digital pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Ultrasonic-Ranger-p-960.html">Ultrasonic Ranger</a>
      </td>
      <td>grove_ultrasonic_ranger</td>
      <td>y</td>
      <td>arg1 - digital pin</td>
    </tr>
   <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-I2C-UV-Sensor-VEML6070.html">UV Sensor</a>
      </td>
      <td>grove_uv_sensor</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-VOC-and-eCO2-Gas-Sensor-SGP3-p-3071.html">VOC and eCO2 Gas Sensor (SGP30)</a>
      </td>
      <td>grove_voc_eco2_gas_sgp30</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Water-Sensor-p-748.html">Water Sensor</a>
      </td>
      <td>grove_water_sensor</td>
      <td>y</td>
      <td>arg1 - analog pin</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-WS2813-RGB-LED-Strip-Waterproof-30-LED-m-1m-p-3124.html">WS2813 RGB LED Strip Waterproof - 30 LED/m - 1m</a><br>
        <a href="https://www.seeedstudio.com/Grove-WS2813-RGB-LED-Strip-Waterproof-60-LED-m-1m-p-3126.html">WS2813 RGB LED Strip Waterproof - 60 LED/m - 1m</a><br>
        <a href="https://www.seeedstudio.com/Grove-WS2813-RGB-LED-Strip-Waterproof-144-LED-m-1m-p-3127.html">WS2813 RGB LED Strip Waterproof - 144 LED/m - 1m</a>
      </td>
      <td>grove_ws2813_rgb_led_strip</td>
      <td>&bull;</td>
      <td>
<pre>
arg1 - PWM pin
arg2 - led count,
       could be 30, 60, 144
       for the three models listed left.
</pre>
      </td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Sunlight-Sensor.html">sunlight Sensor</a>
      </td>
      <td>grove_sunlight_sensor</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Digital-Light-Sensor-TSL2561.htm">digital light Sensor</a>
      </td>
      <td>grove_digital_light_sensor</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://wiki.seeedstudio.com/Grove-Barometer-High-Accuracy/#resources">Grove - Barometer Sensor</a>
      </td>
      <td>Grove_Barometer_hp206f_sensor</td>
      <td>y</td>
      <td>I2C</td>
    </tr>
  </table>
</div>

&bull; means supported but prefix `sudo` must be provided.<br>
`y` means supported without prefix `sudo`.

<br><br>
## GUI (graphical user interface)
You can copy below codes directly, and paste into any Python IDE (such as Thonny Python IDE) to run the demo and see the effect.
### Grove - LED
```python
import time
from grove.grove_led import GroveLed

led = GroveLed(12)

while True:
    led.on()
    time.sleep(1)
    led.off()
    time.sleep(1)
```

### Relay
```python
import time
from grove.factory import Factory

relay = Factory.getGpioWrapper("Relay",16)
while True:
    relay.on()
    time.sleep(2)
    relay.off()
    time.sleep(5)
```

### Button
```python
import time
from grove.factory import Factory

pin = 12
button = Factory.getButton("GPIO-HIGH", pin)

while True:
    if button.is_pressed():
        print('Button is pressed')
    else:
        print('Button is released')
    time.sleep(1)
```

### PIR Motion Sensor
#### mini PIR motion sensor
```python
import time
from grove.factory import Factory

### connect to pin 5(slot D5)
pir = Factory.getGpioWrapper("PIRMotion", 5)
while True:
    if pir.has_motion():
        print("Hi, people is moving")
    else:
        print("Watching")
    time.sleep(1)
```

### Buzzer
```python
import time
from grove.factory import Factory

buzzer = Factory.getGpioWrapper("Buzzer", 12)
while True:
    buzzer.on()
    time.sleep(1)
    buzzer.off()
    time.sleep(3)
```

### Electromagnet
```python
import time
from grove.factory import Factory

magnet = Factory.getGpioWrapper("Electromagnet", 12)
while True:
    magnet.on()
    time.sleep(1)
    magnet.off()
    time.sleep(3)
```

### 4 Digit Display
```python
import time
from grove.grove_4_digit_display import Grove4DigitDisplay

display = Grove4DigitDisplay(12, 13))

count = 0
while True:
    t = time.strftime("%H%M", time.localtime(time.time()))
    display.show(t)
    display.set_colon(count & 1)
    count += 1
    time.sleep(1)
```

### I2C Color Sensor V2
```python
import time
from grove.grove_i2c_color_sensor_v2 import GroveI2cColorSensorV2
sensor = GroveI2cColorSensorV2()

print('Raw data of red-filtered, green-filtered, blue-filtered and unfiltered photodiodes')
while True:
    # r, g, b = sensor.rgb
    r, g, b, clear = sensor.raw
    print((r, g, b, clear))
    time.sleep(1.0)
```

### I2C Motor Driver
Use along with DC-Motor.
```python
import time
from grove.grove_i2c_motor_driver import MotorDriver

motor = MotorDriver()
while True:
    # speed range: 0(lowest) - 100(fastest)
    motor.set_speed(100)
    # channel 1 only
    # to set channel 1&2: motor.set_speed(100, 100)

    # direction: True(clockwise), False(anti-clockwise)
    motor.set_dir(True)
    # channel 1 only,
    # to set channel 1&2: motor.set_dir(True, True)

    time.sleep(2)

    motor.set_speed(70)
    motor.set_dir(False)
    time.sleep(2)
```

<br>
Use along with Stepper Motor 24BYJ48/28BYJ48.

Connections between 24BYJ48/28BYJ48 and I2C-Motor-Driver:
<div>
  <table border="0">
    <tr align="center">
      <th>I2C-Motor-Driver</th>
      <th>24BYJ48 Wires</th>
      <th>28BYJ48 Wires</th>
    </tr>
    <tr align="center">
      <td>J1.M1-/OUT1</td>
      <td>Blue</td>
      <td>Blue</td>
    </tr>
    <tr align="center">
      <td>J1.M1+/OUT2</td>
      <td>Pink</td>
      <td>Orange</td>
    </tr>
    <tr align="center">
      <td>J2.M2-/OUT3</td>
      <td>Orange</td>
      <td>Yellow</td>
    </tr>
    <tr align="center">
      <td>J2.M2+/OUT4</td>
      <td>Yellow</td>
      <td>Pink</td>
    </tr>
  </table>
</div>

```python
from grove.factory import Factory
import time

motor = Factory.getStepperMotor("24BYJ48")
# If it's 28BYJ48
# motor = Factory.getStepperMotor("28BYJ48")
ANGLE = 360 # rotate 360 degrees = 1 cycle
motor.rotate(ANGLE)
# set speed to max rpm, for Motor 24BYJ48, it's 30 RPM.
# direction is clockwise, anti-clockwise use negative value (-30)
SPEED = motor.speed_max # SPEED = 30
motor.speed(SPEED)
motor.enable(True) # enable the motor, begin to run

seconds = ANGLE / 360.0 / SPEED * 60
print("Motor {} rotate {} degrees, time = {:.2f}".format(motor.name, ANGLE, time.time()))
print("      with speed {} RPM".format(SPEED))
print("      will be stop after {:4.2f} seconds".format(seconds))
while True:
    left = motor.rotate()
    if left < 1e-5: break
    print("Angle left {:6.2f} degrees, time = {:.2f}".format(left, time.time()))
    time.sleep(1.0)

print("Motor run ended, time = {:.2f} !".format(time.time()))
```

#### Buzzer PWM mode
```python
import time
import RPi.GPIO as GPIO

# Grove Base Hat for Raspberry Pi
#   PWM JST SLOT - PWM[12 13 VCC GND]
pin = 12
# create PWM instance
pwm = GPIO.PWM(pin, 10)
pwm.start(0) 
# 1000 Hz sound
pwm.ChangeFrequency(1000)
pwm.ChangeDutyCycle(100)
```

### Temperature & Humidity Sensor(DHT11)
```python
import time
from grove.grove_temperature_humidity_sensor import DHT

# DHT11 type
# insert to GPIO pin 5, slot D5
dht11 = DHT("11", 5)

while True:
    humi, temp = dht11.read()
    print('DHT{0}, humidity {1:.1f}%, temperature {2:.1f}*'.format(dht11.dht_type, humi, temp))
    time.sleep(1)
```

### LCD 16x2 Characters
#### OLED Display 1.12"
```python
import time
from grove.factory import Factory

# LCD 16x2 Characters
dobj = Factory.getDisplay("JHD1802")
# If it's OLED Display 1.12"
# dobj = Factory.getDisplay("SH1107G")
rows, cols = dobj.size()
print("LCD model: {}".format(dobj.name))
print("LCD type : {} x {}".format(cols, rows))

dobj.setCursor(0, 0)
dobj.write("hello world!")
dobj.setCursor(0, cols - 1)
dobj.write('X')
dobj.setCursor(rows - 1, 0)
for i in range(cols):
    dobj.write(chr(ord('A') + i))

time.sleep(3)
dobj.clear()
```

### Ultrasonic Ranger
```python
from grove.grove_ultrasonic_ranger import GroveUltrasonicRanger
import time

sonar = GroveUltrasonicRanger(12) # pin12, slot D12

print('Detecting distance...')
while True:
    print('{} cm'.format(sonar.get_distance()))
    time.sleep(1)
```

### sunlight Sensor
```python
from grove.grove_sunlight_sensor import GroveSi115xSensor
import time

SI1151 = GroveSi115xSensor()
while True:
    print('Visible %03d IR %03d' % (SI1151.ReadVisible , SI1151.ReadIR),end=" ")
    print('\r', end='')
    time.sleep(0.5)
```

### digitial ligth sensor
```python
import time
from grove.grove_digital_light_sensor import GroveDigitalLight

sensor = GroveDigitalLight()
while True:
    print('light: %d' %(sensor.read_light_chan0()))
    time.sleep(0.5)
```
