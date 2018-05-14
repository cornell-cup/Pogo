import serial
import time
import struct

#CONSTANTS - SWAP TO CHANGE BEHAVIOR
HasStatusUpdate = True
StatusTime = 500




s0 = serial.Serial('/dev/ttyACM0', 115200)
#s1 = serial.Serial('/dev/ttyACM1', 115200)
#s2 = serial.Serial('/dev/ttyACM2', 115200)
#s3 = serial.Serial('/dev/ttyACM3', 115200)

#Reassign as necessary, plug them in the order you want.
#Use dmesg command in the terminal to determine which one is which.
s_motor = s0
s_imu = s0
s_nunchuck = s0
s_solenoid = s0

imu_connected = False
motor_connected = False
sol_connected = False
nunchuck_connected = False


#Imu Values
imu_euler = (0.0,0.0,0.0)
imu_gyro = (0.0,0.0,0.0)
imu_linaccel = (0.0,0.0,0.0)
#Motor Values
motor_rpm = 0
motor_status = 0     #Status should reflect the heartbeat
               #0 -> Motor Off, 1 -> Motor On, 2 -> Shutoff
motor_set_status = 0 #Status rpi wants the motor to have.
motor_waiting_on_ack = False
motor_waiting_off_ack = False
#Solenoid Values
sol_status = 0
sol_set_status = 0
##sol_period = 0
##sol_duty_cycle = .5
##sol_waiting_on_ack = False
##sol_waiting_off_ack = False

#Nunchuck Values
nun_wheel = 0
nun_jump = 0
nun_x = -1

#Rpi Values #Make sure to normalize by loop time.
rpi_nun_override = False #Use when the rpi needs to directly control motor/solenoid
                         #Reason for the motor_set_status and nun_wheel split.
state = 0
error = 0
theta = 0
theta_prev = 0
theta_deriv = 0
theta_average = 0

current = 0

#PID Constants
zero_offset = 0



#Time Keeping
millis = lambda: int(round(time.time() * 1000))
cur_time = millis()
start_time = cur_time
prev_time = cur_time
status_time = cur_time
motor_ack_timer = cur_time



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
        
def printStatus():
  print( "-------------{}--------------".format( cur_time - start_time ) )
  print( "Motor Status: {} :: Set-> {} :: Current -> {}".format( motor_status, motor_set_status, current ) )
  print( "Solenoid Status: {} :: Set-> {}".format( sol_status, sol_set_status ) )


def processMotorPacket(packet):
  global motor_status
  if not packet:
    print ("Error: Packet Empty")
  elif len(packet) == 0:
    print ("Error: Packet Empty")
  elif packet[0] == '0':
    print ("Emergency motor shutdown!!!")
    motor_status = 2
  #heartbeat #Todo need to check when the motor_status does not equal what rpi says it should be.
  elif packet[0] == '1' and len(packet)==2:
    if packet[1] == '0':
      motor_status = 2
    if packet[1] == '1':
      motor_status = 1
    if packet[1] == '2':
      motor_status = 0
  #motor acks
  elif packet[0] == '2' and len(packet)==2:
    if packet[1] == '0':
      print('Motor turned off')
      global motor_waiting_off_ack
      if motor_waiting_off_ack:
        motor_off_ack = False
        motor_status = 0
      else:
        print("Received ack wrongly motor_off")
    if packet[1] == '1':
      print('Motor turned on')
      global motor_waiting_on_ack
      if motor_waiting_on_ack:
        motor_on_ack = False
        motor_status = 1
      else:
        print("Received ack wrongly motor_on")

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
    global motor_set_status
    global nun_jump
    global sol_set_status
    global nun_x
    nun_wheel = packet[0] - 48 #Todo Should probably fix this arduino side.
    if ( not rpi_nun_override and nun_wheel != motor_set_status ):
      motor_set_status = nun_wheel  
    nun_jump = packet[1] - 48
    if ( not rpi_nun_override and nun_jump != sol_set_status ):
      sol_set_status = nun_jump  
    nun_x = int.from_bytes(packet[2:6], byteorder='little', signed=False)
    

processImuSerial = ProcessSerial("imu", s_imu, processImuPacket)
processMotorSerial = ProcessSerial("motor", s_motor, processMotorPacket)
processNunchuckSerial = ProcessSerial("nunchuck", s_nunchuck, processNunchuckPacket)

SerialReaders = []
if imu_connected:
  SerialReaders.append(processImuSerial)
if motor_connected:
  SerialReaders.append(processMotorSerial)
if sol_connected:
  SerialReaders.append(processSolenoidSerial)
if nunchuck_connected:
  SerialReaders.append(processNunchuckSerial)
  
print ( "Starting" )

while 1:
  #Timers
  prev_time = time
  cur_time = millis()
  
  #read from devices to update info
  for sr in SerialReaders:
    sr.readSerial()
  #use info to calculate what to return

  if ( HasStatusUpdate and (cur_time - status_time) > StatusTime ):
    status_time = cur_time
    printStatus()
    

  #calculate PID loop
  

  #write serial to devices
  if motor_connected:
    if ( motor_status == 0 and motor_set_status == 1 and not motor_waiting_on_ack):
      #Send Motor On Packet
      s_motor.write(b'\x02')
      s_motor.write(b'\x02')
      s_motor.write(b'\x01')
      s_motor.write(b'\x01')
      motor_waiting_on_ack = True
      
    if ( motor_status == 1 and motor_set_status == 0 and not motor_waiting_off_ack):
      #Send Motor On Packet
      s_motor.write(b'\x02')
      s_motor.write(b'\x02')
      s_motor.write(b'\x01')
      s_motor.write(b'\x00')
      motor_waiting_off_ack = True

    # Write motor current every loop if motor should be on.
    if ( motor_set_status == 1 ):
      s_motor.write(b'\x02')
      s_motor.write(b'\x05')
      s_motor.write(b'\x02')
      s_motor.write(bytearray(struct.pack('f', current)))
    

  time.sleep(.005)
