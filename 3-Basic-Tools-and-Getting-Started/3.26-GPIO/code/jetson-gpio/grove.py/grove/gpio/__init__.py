
try:
    from grove.gpio.gpio_rpi import GPIO
except ImportError:
    print('RPi.GPIO is not available.Exit.')
    exit(1)

from grove.gpio.wrapper import GPIOWrapper

__all__ = ['GPIO', 'GPIOWrapper']
