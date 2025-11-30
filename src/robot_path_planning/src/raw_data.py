#!/usr/bin/env python

import serial
import time

lidar = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

lidar.write(b'\xA5\x20')

while True:
    data = lidar.read(128)

    if data:
        print(data)
