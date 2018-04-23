import serial
import time

s = serial.Serial('/dev/ttyACM0')
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
		time.sleep(1)
		while s.inWaiting() > 0:
			out+=s.read(1)
		
		if out != '':
			print ">>" + out

s.close()
