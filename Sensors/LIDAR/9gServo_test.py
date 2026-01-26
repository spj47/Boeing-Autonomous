import serial
import time

ser = serial.Serial('/dev/tty.usbserial-1420', 9600)
time.sleep(2)  # allow Arduino reset to establish connection

while True:
    angle = input("Enter angle (0–180): ")
    ser.write(f"{angle}\n".encode())
    ser.flush() # flushing here reduces servo jitter with unreliable power