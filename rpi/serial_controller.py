import serial
import time

s0 = serial.Serial('/dev/ttyACM0')
s1 = serial.Serial('/dev/ttyACM1')

#Reassign as necessary, plug them in the order you want.
s_motor = s0
s_imu = s1

processMotorSerial = new ProcessSerial("motor", s_motor, processMotorPacket)

def processMotorPacket(packet):
    if !packet:
        print "Error: Packet Empty"
    elif len(packet) == 0:
        print "Error: Packet Empty"
    elif packet[0] == 'Q':
        print "Motor shutdown packet"
    else:
        print ''.join(packet)

def processImuPacket(packet):
    if !packet:
        print "Error: Packet Empty"
    elif len(packet) == 0:
        print "Error: Packet Empty"
    elif packet[0] == 'Q':
        print "Motor shutdown packet"
    else:
        print ''.join(packet)


class ProcessSerial():


    def __init__(self, name, port, processPacket):
        self.name = name
        self.port = port
        self.processPacket = processPacket
        self.buffer = []
        self.hasStart = false
        self.hasEnd = false
        
        
    def readSerial():
	while s.inWaiting() > 0:
	    inByte = s.read(1)
	    if inByte == '\2':
                self.hasStart = true
                self.buffer = []
                
                
        




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
