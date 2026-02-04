import time
import serial
import RPi.GPIO as GPIO
from tkinter import Tk, Label, Button
import minimalmodbus as mm

traffic_light_1 = mm.Instrument('/dev/ttyAMA0', 1)
barrier_1 = mm.Instrument('/dev/ttyAMA0', 2)
road_sign_1 = mm.Instrument('/dev/ttyAMA0', 3)

for device in [traffic_light_1, barrier_1, road_sign_1]:
    device.serial.baudrate = 9600
    device.serial.bytesize = 8
    device.serial.parity   = minimalmodbus.serial.PARITY_NONE
    device.serial.stopbits = 1
    device.serial.timeout  = 0.5
    device.mode = minimalmodbus.MODE_RTU
    device.clear_buffers_before_each_transaction = True

def poll_devices():
    try:
        val1 = traffic_light_1.write_register(0, 0)   # (address, number of decimals)
        print(f"Device 1: {val1}")
    except Exception as e:
        print(f"Device 1 error: {e}")

    try:
        val2 = barrier_1.read_register(0, 0)
        print(f"Device 2: {val2}")
    except Exception as e:
        print(f"Device 2 error: {e}")

    try:
        val3 = road_sign_1.read_register(0, 0)
        print(f"Device 3: {val3}")
    except Exception as e:
        print(f"Device 3 error: {e}")

# Основной цикл
while True:
    poll_devices()
    time.sleep(1)