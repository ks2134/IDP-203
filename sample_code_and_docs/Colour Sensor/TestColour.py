from machine import Pin
from machine import I2C
from tcs34725 import TCS34725
from time import sleep_ms

i2c_bus = I2C(0, sda=Pin(16), scl=Pin(17))
tcs = TCS34725(i2c_bus)

print('raw: {}'.format(tcs.read('raw')))

print('rgb: {}'.format(tcs.read('rgb')))
