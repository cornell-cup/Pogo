import serial
import time
import struct

s0 = serial.Serial('/dev/ttyACM0', 115200)
#s1 = serial.Serial('/dev/ttyACM1')

#Reassign as necessary, plug them in the order you want.
#Use dmesg command in the terminal to determine which one is which.
s_motor = s0
s_imu = s0
s_nunchuck = s0

#Imu Values
imu_euler = (0.0,0.0,0.0)
imu_gyro = (0.0,0.0,0.0)
imu_linaccel = (0.0,0.0,0.0)
#Motor Values
motor_rpm = 0
status = 0     #Status should reflect the heartbeat
               #0 -> Motor Off, 1 -> Motor On, 2 -> Shutoff
set_status = 0 #Status rpi wants the motor to have.motor_rpm = 0
#Nunchuck Values
nun_wheel = 0
nun_jump = 0
nun_x = -1

class ProcessSerial:

  #processPacket is a function
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
      elif len(self.buffer) > 100:
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
  elif packet[0] == '0':
    print ("Motor shutdown packet")

  #heartbeat
  elif packet[0] == '1' and len(packet)==2:
    if packet[1] == '0':
      print('Emergency Shutoff')
    if packet[1] == '1':
      print('Motor On')
    if packet[1] == '2':
      print('Motor Off')

  elif packet[0] == '2' and len(packet)==2:
    if packet[1] == '0':
      print('Motor turned off')
    if packet[1] == '1':
      print('Motor turned on')

  elif packet[0] == '3' and len(packet) == 5:
    global motor_rpm
    #figure out if line 97 works, might have to conver to byte list
    motor_rpm = int.from_bytes(packet[1:5], byteorder='little', signed=False)
  else:
    print (packet)

def processImuPacket(packet):
  if not packet:
    print ("Error: Packet Empty")
  elif len(packet) == 0:
    print ("Error: Packet Empty")
  elif packet[0] == '\x45':
    print ("Imu Error")
  elif packet[0] == 1 and len(packet) == 73:
    global imu_euler
    global imu_gyro
    global imu_linaccel
    euler_x = struct.unpack('d', bytes(packet[1:9]))
    euler_y = struct.unpack('d', bytes(packet[9:17]))
    euler_z = struct.unpack('d', bytes(packet[17:25]))
    imu_euler = (euler_x,euler_y,euler_z)
    gyro_x = struct.unpack('d', bytes(packet[25:33]))
    gyro_y = struct.unpack('d', bytes(packet[33:41]))
    gyro_z = struct.unpack('d', bytes(packet[41:49]))
    imu_gyro = (gyro_x,gyro_y,gyro_z)
    linacc_x = struct.unpack('d', bytes(packet[49:57]))
    linacc_y = struct.unpack('d', bytes(packet[57:65]))
    linacc_z = struct.unpack('d', bytes(packet[65:73]))
    imu_linaccel = (linacc_x,linacc_y,linacc_z)

  else:
    print (packet)

def processNunchuckPacket(packet):
  if not packet:
    print ("Nunchuck Error: Packet Empty")
  elif len(packet) != 6:
    print ("Nunchuck Error: Incorrect Packet Size") 
  else:
    global nun_wheel
    global nun_jump
    global nun_x
    nun_wheel = int.from_bytes(packet[0:1], byteorder='little', signed=False)
    nun_jump = int.from_bytes(packet[1:2], byteorder='little', signed=False)
    nun_x = int.from_bytes(packet[2:6], byteorder='little', signed=False)
    



    
processMotorSerial = ProcessSerial("motor", s_motor, processMotorPacket)
processImuSerial = ProcessSerial("imu", s_imu, processImuPacket)
processNunchuckSerial = ProcessSerial("nunchuck", s_nunchuck, processNunchuckPacket)

SerialReaders = []
#SerialReaders.append(processMotorSerial)
#SerialReaders.append(processImuSerial)
SerialReaders.append(processNunchuckSerial)
#SerialReaders.append(processSolenoidSerial)

print ( "Starting" )

while 1:
  #read from devices to update info
  for sr in SerialReaders:
    sr.readSerial()
  #use info to calculate what to return
  if nun_x != -1:
    print (nun_x)

  time.sleep(.005)
  #write serial to devices
  

