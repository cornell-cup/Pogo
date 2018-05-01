import serial
import time

s = serial.Serial('/dev/ttyACM1')
print(s.isOpen())
print(s.inWaiting())

while 1:
	input = raw_input(">> ")
	if input == 'exit':
		s.close()
		exit()
	else:
		s.write(input)
		out = ''
		while s.inWaiting() == 0:
			time.sleep(.001)
		while s.inWaiting() > 0:
			out+=s.read(1)
		
		if out != '':
			print ">>" + out

s.close()
