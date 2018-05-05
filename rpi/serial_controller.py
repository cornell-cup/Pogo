import serial
import time

s0 = serial.Serial('/dev/ttyACM0')
s1 = serial.Serial('/dev/ttyACM1')

#Reassign as necessary, plug them in the order you want.
s_motor = s0
s_imu = s1

imu_rotation = (0.0,0.0,0.0)
imu_position = (0.0,0.0,0.0)
motor_rpm = 0

processMotorSerial = new ProcessSerial("motor", s_motor, processMotorPacket)
processImuSerial = new ProcessSerial("imu", s_imu, processImuPacket)

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
        print "Imu shutdown packet"
    else:
        print ''.join(packet)


class ProcessSerial():


    def __init__(self, name, port, processPacket):
        self.name = name
        self.port = port
        self.processPacket = processPacket
        self.buffer = []
        self.hasStart = false
        
        
    def readSerial():
	while self.port.inWaiting() > 0:
	    inByte = self.port.read(1)
	    if inByte == '\2':
                self.hasStart = true
                self.buffer = []
            elif inByte == '\3':
                self.processPacket(self.buffer)
                self.reset()
            else:
                if !self.hasStart:
                    1==1
                #the packet is too long, longer than any of our set packets
                elif len(self.buffer) > 10:
                    self.reset()
                else:
                    self.buffer += inByte


    def reset():
        self.buffer = []
        self.hasStart = false
        
                
                
        




while 1:
    #read from devices to update info
    #use info to calculate what to return
    #write serial to devices



    

s.close()
