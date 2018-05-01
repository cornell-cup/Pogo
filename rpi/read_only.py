import serial
import time

s0 = serial.Serial('/dev/ttyACM0')
s1 = serial.Serial('/dev/ttyACM1')
print(s0.isOpen())
print(s0.inWaiting())
print(s1.isOpen())
print(s1.inWaiting())
while 1:
	#print(s0.readline())
	print(s1.readline())
	time.sleep(.1)
s0.close()
s1.close()
