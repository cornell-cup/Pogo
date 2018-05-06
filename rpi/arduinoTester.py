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
global test_val
test_val = 0.0
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
    self.packetSize = -1
    
    
  def readSerial(self):
    inByte = '0'
    while self.port.inWaiting() > 0:
      inByte = self.port.read(1)
      #print(inByte)
      
      if not self.hasStart:
        if inByte == b'\x02':
          self.reset()
          self.hasStart = True
        else:
          print ("Has noise")
      #the packet is too long, longer than any of our set packets
      elif len(self.buffer) > 20:
        self.reset()
      elif self.packetSize == -1:
        self.packetSize = int.from_bytes(inByte, byteorder='little', signed=False)
      else: 
        self.buffer += inByte
        
      if self.packetSize != -1 and len(self.buffer) == self.packetSize :
        self.processPacket(self.buffer)
        self.reset()
      


  def reset(self):
    self.buffer = bytearray()
    self.hasStart = False
    self.packetSize = -1

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
  elif packet[0] == 1 and len(packet) == 9:
    global imu_position
    global imu_rotation
    global test_val
    d = bytes(packet[1:9])
    test = struct.unpack('d', d)
    #print (test)
    test_val = test
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
  int_input = input("Input an int >> ")
  float_input = input ("Input a float >> ")
  if int_input == 'exit':
    s0.close()
    exit()
  else:
    
    a = int(int_input).to_bytes(4, byteorder='little')
    b = float(float_input)
    print(b)
    bb = bytearray(struct.pack('d', b))
    s0.write(b'\x02')
    s0.write(b'\x0C')
    s0.write(a)
    s0.write(bb)
    out = bytearray()
    #while s0.inWaiting() == 0:
    time.sleep(1.001)
    while s0.inWaiting() > 0:
      out+=s0.read(1)
      
		
    print (out)
    print (">>" + out[14::].decode("utf-8"))


for sr in SerialReaders:
  sr.closePort()
