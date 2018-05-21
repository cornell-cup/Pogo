import serial
import time

s0 = serial.Serial('/dev/pts/3')
s1 = serial.Serial('/dev/pts/2')
print(s0.isOpen())
print(s0.inWaiting())
#print(s1.isOpen())
#print(s1.inWaiting())
while 1:
	s1.write(b'0x02')
	print(s0.read())
	#print(s1.readline())
	time.sleep(.1)
s0.close()
#s1.close()
