# 3.32 Super GPIO

> [!IMPORTANT]
> This page is intended for the Seeed `reComputer Super` carrier-board family, such as [`reComputer Super J4011`](https://www.seeedstudio.com/reComputer-Super-J401-Carrier-Board-p-6642.html). The 40-pin header layout and pin behavior are board-specific and should not be assumed to match other Jetson carrier boards.

## Introduction

GPIO stands for General Purpose Input/Output. These pins let software read external digital signals or drive simple peripherals such as LEDs, buttons, buzzers, and control lines. The reComputer Super features a 40-pin extension header that provides access to GPIO pins, allowing for easy integration with external devices.

![40-Pin Extension Header](./images/super-gpio-40pin.jpg)

## Hardware Connection

The reComputer Super features a 40-pin extension header that provides access to GPIO pins. To use the GPIO pins, you need to connect external devices to the appropriate pins on this header.

### 40-Pin Header Pinout

The detail of 40-pin header is shown below:

<div class="table-center">
<table style={{textAlign: 'center'}}>
<thead>
<tr>
  <th>Header Pin</th>
  <th>Signal</th>
  <th>BGA Pin</th>
  <th>Default Function</th>
</tr>
</thead>
<tbody>
<tr><td>1</td><td>3.3V</td><td>-</td><td>Main 3.3V Supply</td></tr>
<tr><td>2</td><td>5V</td><td>-</td><td>Main 5V Supply</td></tr>
<tr><td>3</td><td>I2C1_SDA</td><td>PDD.02</td><td>I2C #1 Data</td></tr>
<tr><td>4</td><td>5V</td><td>-</td><td>Main 5V Supply</td></tr>
<tr><td>5</td><td>I2C1_SCL</td><td>PDD.01</td><td>I2C #1 Clock</td></tr>
<tr><td>6</td><td>GND</td><td>-</td><td>Ground</td></tr>
<tr><td>7</td><td>GPIO09</td><td>PAC.06</td><td>General Purpose I/O</td></tr>
<tr><td>8</td><td>UART1_TXD</td><td>PR.02</td><td>UART #1 Transmit</td></tr>
<tr><td>9</td><td>GND</td><td>-</td><td>Ground</td></tr>
<tr><td>10</td><td>UART1_RXD</td><td>PR.03</td><td>UART #1 Receive</td></tr>
<tr><td>11</td><td>UART1_RTS</td><td>PR.04</td><td>UART #1 Request to Send</td></tr>
<tr><td>12</td><td>I2S0_SCLK</td><td>PH.07</td><td>Audio I2S #0 Clock</td></tr>
<tr><td>13</td><td>SPI1_SCK</td><td>PY.00</td><td>SPI #1 Clock</td></tr>
<tr><td>14</td><td>GND</td><td>-</td><td>Ground</td></tr>
<tr><td>15</td><td>GPIO12</td><td>PN.01</td><td>General Purpose I/O</td></tr>
<tr><td>16</td><td>SPI1_CS1</td><td>PY.04</td><td>SPI #1 Chip Select #1</td></tr>
<tr><td>17</td><td>3.3V</td><td>-</td><td>Main 3.3V Supply</td></tr>
<tr><td>18</td><td>SPI1_CS0</td><td>PY.03</td><td>SPI #1 Chip Select #0</td></tr>
<tr><td>19</td><td>SPI0_MOSI</td><td>PZ.05</td><td>SPI #0 Master Out / Slave In</td></tr>
<tr><td>20</td><td>GND</td><td>-</td><td>Ground</td></tr>
<tr><td>21</td><td>SPI0_MISO</td><td>PZ.04</td><td>SPI #0 Master In / Slave Out</td></tr>
<tr><td>22</td><td>SPI1_MISO</td><td>PY.01</td><td>SPI #1 Master In / Slave Out</td></tr>
<tr><td>23</td><td>SPI0_SCK</td><td>PZ.03</td><td>SPI #0 Clock</td></tr>
<tr><td>24</td><td>SPI0_CS0</td><td>PZ.06</td><td>SPI #0 Chip Select #0</td></tr>
<tr><td>25</td><td>GND</td><td>-</td><td>Ground</td></tr>
<tr><td>26</td><td>SPI0_CS1</td><td>PZ.07</td><td>SPI #0 Chip Select #1</td></tr>
<tr><td>27</td><td>ID_I2C_SDA (I2C0_SDA)</td><td>PDD.00</td><td>I2C #0 Data</td></tr>
<tr><td>28</td><td>ID_I2C_SCL (I2C0_SCL)</td><td>PCC.07</td><td>I2C #0 Clock</td></tr>
<tr><td>29</td><td>GPIO01</td><td>PQ.05</td><td>General Purpose I/O</td></tr>
<tr><td>30</td><td>GND</td><td>-</td><td>Ground</td></tr>
<tr><td>31</td><td>GPIO11</td><td>PQ.06</td><td>General Purpose I/O</td></tr>
<tr><td>32</td><td>GPIO07</td><td>PG.06</td><td>General Purpose I/O</td></tr>
<tr><td>33</td><td>GPIO13</td><td>PG.00</td><td>System Reserved</td></tr>
<tr><td>34</td><td>GND</td><td>-</td><td>Ground</td></tr>
<tr><td>35</td><td>I2S0_LRCK (I2S0_FS)</td><td>PI.02</td><td>Audio I2S #0 Frame Sync</td></tr>
<tr><td>36</td><td>UART1_CTS</td><td>PR.05</td><td>UART #1 Clear to Send</td></tr>
<tr><td>37</td><td>SPI1_MOSI</td><td>PY.02</td><td>SPI #1 Master Out / Slave In</td></tr>
<tr><td>38</td><td>I2S0_SDIN (I2S0_DIN)</td><td>PI.01</td><td>Audio I2S #0 Data In</td></tr>
<tr><td>39</td><td>GND</td><td>-</td><td>Ground</td></tr>
<tr><td>40</td><td>I2S0_SDOUT (I2S0_DOUT)</td><td>PI.00</td><td>Audio I2S #0 Data Out</td></tr>
</tbody>
</table>
</div>

## GPIO Library Installation

The `Jetson.GPIO` library is required to work with GPIO pins on the reComputer Super. If you haven't installed it yet, you can do so by following the instructions in [3.27 Installing the GPIO Library](../3.27-Installing-the-GPIO-Library/README.md).

## Numbering Modes

The `Jetson.GPIO` library supports two common numbering schemes:

| Mode | Description | Typical Use |
| --- | --- | --- |
| `BOARD` | Numbers pins by their physical location on the 40-pin header | Best when you are wiring directly from the header silkscreen or a pinout diagram |
| `BCM` | Numbers pins by the GPIO mapping used by the library | Best when you are following Python GPIO examples that refer to logical GPIO IDs |

## GPIO Usage Examples

### Basic GPIO Output

```python
import Jetson.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BOARD)

# Define pin
output_pin = 12

# Set up the pin
GPIO.setup(output_pin, GPIO.OUT)

try:
    while True:
        # Turn on the pin
        GPIO.output(output_pin, GPIO.HIGH)
        time.sleep(1)
        # Turn off the pin
        GPIO.output(output_pin, GPIO.LOW)
        time.sleep(1)
except KeyboardInterrupt:
    # Clean up
    GPIO.cleanup()
```

### Basic GPIO Input

```python
import Jetson.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BOARD)

# Define pin
input_pin = 11

# Set up the pin
GPIO.setup(input_pin, GPIO.IN)

try:
    while True:
        # Read the pin value
        value = GPIO.input(input_pin)
        print(f"Pin value: {value}")
        time.sleep(0.5)
except KeyboardInterrupt:
    # Clean up
    GPIO.cleanup()
```

## Super-Specific GPIO Features

The reComputer Super's 40-pin header provides access to a range of GPIO pins, allowing for flexible integration with external devices. This makes it ideal for a variety of applications, including:

- Controlling LEDs and other indicators
- Reading button presses and sensor inputs
- Controlling motors and actuators
- Interfacing with other microcontrollers and devices

## Safety Precautions

- Always double-check pin connections before applying power
- Use appropriate resistors when connecting LEDs and other components
- Avoid short-circuiting GPIO pins
- Be mindful of the maximum current rating for GPIO pins

## Further Reading

- [Jetson.GPIO Library Documentation](https://github.com/NVIDIA/jetson-gpio)
- [reComputer Super Hardware and Interfaces Usage](https://wiki.seeedstudio.com/recomputer_jetson_super_hardware_interfaces_usage/)
