from machine import Pin
from time import sleep

sensor = Pin(15, Pin.IN, Pin.PULL_DOWN)

while True:
    print(sensor.value())
    sleep(0.5)