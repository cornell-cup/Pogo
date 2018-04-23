import serial
import time

s = serial.Serial('/dev/ttyACM0')
print(s.isOpen())
print(s.inWaiting())

while 1:
	print(s.readline())
s.close()
