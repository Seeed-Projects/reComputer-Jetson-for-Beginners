
# Install Dependencies

- Add repository

```shell
# RPi
echo "deb https://seeed-studio.github.io/pi_repo/ stretch main" | sudo tee /etc/apt/sources.list.d/seeed.list
```
- Add public GPG key

```shell
curl https://seeed-studio.github.io/pi_repo/public.key | sudo apt-key add -
# or
# sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys BB8F40F3
```

### Enable the I2C interface.
```shell
sudo raspi-config
```
- Select Interfacing Options > I2C > Yes > Ok > Finish
- Enable the I2C interface.

### In later versions of Python 3, it is recommended to use a virtualenv for isolated package management
```shell
sudo apt install python3-virtualenv
virtualenv -p python3 env
source env/bin/activate
```

#### Install library raspberry-gpio-python for RPi
```shell
sudo apt update
# python3
sudo apt install python3-rpi.gpio
```

#### Install library rpi_ws281x for RPi
```shell
# python3
sudo pip3 install rpi_ws281x
# env
pip install rpi_ws281x
```

<br><br>
# Install grove.py
From PyPI

> Note: This method could not get the lastest code most time.

```shell
# python3
sudo pip3 install grove.py
# env
pip install grove.py
```
