'''
## License

The MIT License (MIT)

Grove Base Hat for the Raspberry Pi, used to connect grove sensors.
Copyright (C) 2018  Seeed Technology Co.,Ltd.
'''

import io
from setuptools import setup, find_packages


with open('README.md') as f:
    long_description = f.read()

requirements = ['smbus2', 'bmm150', 'bme680', 'sgp30']


def is_pi():
    found = False
    try:
        with open('/proc/cpuinfo', 'r') as cpuinfo:
            for line in cpuinfo:
                if line.startswith('Model'):
                    _, value = line.strip().split(':', 1)
                    value = value.strip()
                    if value.startswith('Raspberry Pi'):
                        found = True
                        break  
    except IOError:
        pass

    return found


if is_pi():
    requirements.append('RPi.GPIO')
    requirements.append('rpi_ws281x')


setup(
    name='Seeed-grove.py',
    version='0.7',
    description='Python library for Seeedstudio Grove devices',
    long_description=long_description,
    long_description_content_type='text/markdown',
    author='Seeed',
    author_email='ruiqian.tang@seeed.cc',
    url='https://github.com/seeed-studio/grove.py',
    packages=find_packages(include=['grove']),
    include_package_data=True,
    install_requires=requirements,
    entry_points={
        'console_scripts': [
            'grove_1wire_thermocouple_amplifier_max31850=grove.grove_1wire_thermocouple_amplifier_max31850:main',
            'grove_3_axis_accelerometer_adxl372=grove.grove_3_axis_accelerometer_adxl372:main',
            'grove_3_axis_compass_bmm150=grove.grove_3_axis_compass_bmm150:main',
            'grove_3_axis_digital_accelerometer=grove.grove_3_axis_digital_accelerometer:main',
            'grove_4_digit_display=grove.grove_4_digit_display:main',
            'grove_6_axis_accel_gyro_bmi088=grove.grove_6_axis_accel_gyro_bmi088:main',
            'grove_12_key_cap_i2c_touch_mpr121=grove.grove_12_key_cap_i2c_touch_mpr121:main',
            'grove_16x2_lcd=grove.display.jhd1802:main',
            'grove_air_quality_sensor_v1_3=grove.grove_air_quality_sensor_v1_3:main',
            'grove_button=grove.grove_button:main',
            'grove_cap_touch_slider_cy8c=grove.grove_cap_touch_slider_cy8c:main',
            'grove_collision_sensor=grove.grove_collision_sensor:main',
            'grove_gesture_sensor=grove.grove_gesture_sensor:main',
            'grove_high_accuracy_temperature=grove.grove_high_accuracy_temperature:main',
            'grove_i2c_color_sensor_v2=grove.grove_i2c_color_sensor_v2:main',
            'grove_i2c_motor_driver=grove.grove_i2c_motor_driver:main',
            'grove_i2c_thermocouple_amplifier_mcp9600=grove.grove_i2c_thermocouple_amplifier_mcp9600:main',
            'grove_imu_9dof_icm20600_ak09918=grove.grove_imu_9dof_icm20600_ak09918:main',
            'grove_lcd_1.2inches=grove.display.sh1107g:main',
            'grove_led=grove.grove_led:main',  
            'grove_light_sensor_v1_2=grove.grove_light_sensor_v1_2:main',
            'grove_loudness_sensor=grove.grove_loudness_sensor:main',
            'grove_mech_keycap=grove.grove_mech_keycap:main',
            'grove_mini_pir_motion_sensor=grove.grove_mini_pir_motion_sensor:main',  
            'grove_moisture_sensor=grove.grove_moisture_sensor:main',     
            'grove_multi_switch=grove.grove_multi_switch:main',
            'grove_multi_switch_poll=grove.button.button_i2c:main',
            'grove_oled_display_128x64=grove.grove_oled_display_128x64:main',                   
            'grove_optical_rotary_encoder=grove.grove_optical_rotary_encoder:main',
            'grove_piezo_vibration_sensor=grove.grove_piezo_vibration_sensor:main',
            'grove_pwm_buzzer=grove.grove_pwm_buzzer:main',
            'grove_recorder_v3_0=grove.grove_recorder_v3_0:main',
            'grove_relay=grove.grove_relay:main',
            'grove_gpio=grove.grove_gpio:main',
            'grove_rotary_angle_sensor=grove.grove_rotary_angle_sensor:main',
            'grove_round_force_sensor=grove.grove_round_force_sensor:main',
            'grove_ryb_led_button=grove.grove_ryb_led_button:main',
            'grove_servo=grove.grove_servo:main',
            'grove_slide_potentiometer=grove.grove_slide_potentiometer:main',
            'grove_sound_sensor=grove.grove_sound_sensor:main',
            'grove_step_counter_bma456=grove.grove_step_counter_bma456:main',
            'grove_switch=grove.grove_switch:main',
            'grove_temperature_humidity_bme680=grove.grove_temperature_humidity_bme680:main',
            'grove_temperature_humidity_sht31=grove.grove_temperature_humidity_sensor_sht3x:main',
            'grove_temperature_sensor=grove.grove_temperature_sensor:main',
            'grove_thumb_joystick=grove.grove_thumb_joystick:main',
            'grove_tilt_switch=grove.grove_tilt_switch:main',
            'grove_time_of_flight_distance=grove.grove_time_of_flight_distance:main',
            'grove_touch_sensor=grove.grove_touch_sensor:main',
            'grove_ultrasonic_ranger=grove.grove_ultrasonic_ranger:main',
            'grove_uv_sensor=grove.grove_uv_sensor:main',
            'grove_water_sensor=grove.grove_water_sensor:main',
            'grove_ws2813_rgb_led_strip=grove.grove_ws2813_rgb_led_strip:main',
            'grove_current_sensor=grove.grove_current_sensor:main'
        ],
    },
    zip_safe=False,
    license = 'MIT',
    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Operating System :: POSIX :: Linux',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
    ],
    keywords=['grove'])
