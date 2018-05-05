import serial
import time
import struct

s0 = serial.Serial('/dev/ttyACM0')
#s1 = serial.Serial('/dev/ttyACM1')

#Reassign as necessary, plug them in the order you want.
#Use dmesg command in the terminal to determine which one is which.
s_motor = s0
s_imu = s0

global imu_position
global imu_rotation
imu_rotation = (0.0,0.0,0.0)
imu_position = (0.0,0.0,0.0)
motor_rpm = 0

class ProcessSerial:

  def __init__(self, name, port, processPacket):
    self.name = name
    self.port = port
    self.processPacket = processPacket
    self.buffer = bytearray()
    self.hasStart = False
    
    
  def readSerial(self):
    inByte = '0'
    while self.port.inWaiting() > 0:
      inByte = self.port.read(1)
      print(inByte)
      if inByte == b'\x02':
        print("test")
        self.hasStart = True
        self.buffer = bytearray()
      elif inByte == b'\x03':
        print(len(self.buffer))
        self.processPacket(self.buffer)
        self.reset()
      else:
        if not self.hasStart:
          1==1
          #the packet is too long, longer than any of our set packets
        elif len(self.buffer) > 20:
          self.reset()
        else:
          self.buffer += inByte


  def reset(self):
    self.buffer = bytearray()
    self.hasStart = False

  def closePort(self):
    self.port.close()
        
        


def processMotorPacket(packet):
  if not packet:
    print ("Error: Packet Empty")
  elif len(packet) == 0:
    print ("Error: Packet Empty")
  elif packet[0] == 'Q':
    print ("Motor shutdown packet")
  elif packet[0] == 'R' and len(packet) == 2:
    global motor_rpm
    motor_rpm = packet[1]
  else:
    print (packet)

def processImuPacket(packet):
  if not packet:
    print ("Error: Packet Empty")
  elif len(packet) == 0:
    print ("Error: Packet Empty")
  elif packet[0] == 'Q':
    print ("Imu shutdown packet")
  elif True or packet[0] == 'R' and len(packet) == 7:
    global imu_position
    global imu_rotation
    for byteval in packet:
      print ('%02x' % byteval, end="")
    print ("")
    d = bytes(packet)
    print(d)
    test = struct.unpack('d', d)
    print (test)
    imu_position = (packet[0],packet[0],packet[0])
    imu_rotation = (packet[0],packet[0],packet[0])
  else:
    print (packet)



    
processMotorSerial = ProcessSerial("motor", s_motor, processMotorPacket)
processImuSerial = ProcessSerial("imu", s_imu, processImuPacket)

SerialReaders = []
#SerialReaders += processMotorSerial
SerialReaders.append(processImuSerial)
#SerialReaders += processRadioSerial
#SerialReaders += processSolenoidSerial

print ( "Starting" )

while 1:
  #read from devices to update info
  for sr in SerialReaders:
    sr.readSerial()
  #use info to calculate what to return

  #print (imu_position)
  #print imu_rotation
  #write serial to devices
  


  

for sr in SerialReaders:
  sr.closePort()
