
from machine import I2C, Pin
import time
import math
import bme280

I2C1_SDA = Pin(03)
I2C1_SCL = Pin(02)

i2c1 = I2C(0, scl=I2C1_SCL, sda=I2C1_SDA, freq=100000, timeout=200000 )

env = bme280.BME280(i2c1)

while True:
    time.sleep(0.5)
    temp, press, humid, alt = env.values 
    print("Temperature :\t", temp,"\t°C")
    print("Pressure    :\t", press,"\thPa")
    print("Humidity    :\t", humid,"\t%")
    print("Altitude D  :\t", alt,"\tm")
    print()