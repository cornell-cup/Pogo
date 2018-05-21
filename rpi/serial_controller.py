import serial
import time
import struct
import signal

#CONSTANTS - SWAP TO CHANGE BEHAVIOR
HasStatusUpdate = True
StatusTime = 500
MAX_CURRENT = 5
MAX_RPM = 2200

running = True
def signal_handler(signal, frame):
  global running
  running = False
signal.signal(signal.SIGINT, signal_handler)

s0 = serial.Serial('/dev/ttyACM4', 115200)
s1 = serial.Serial('/dev/ttyACM5', 115200)
s2 = serial.Serial('/dev/ttyACM3', 115200)
#s3 = serial.Serial('/dev/ttyACM3', 115200)

#Reassign as necessary, plug them in the order you want.
#Use dmesg command in the terminal to determine which one is which.
s_imu = s0
s_motor = s1
s_nunchuck = s2
s_solenoid = s0

imu_connected = True
motor_connected = True
sol_connected = False
nunchuck_connected = True


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
nun_requires_press = True

#Rpi Values #Make sure to normalize by loop time.
rpi_nun_override = False #Use when the rpi needs to directly control motor/solenoid
                         #Reason for the motor_set_status and nun_wheel split.
in_shutdown = False
state = 0
theta = 0
theta_prev = 0
theta_deriv = 0
theta_integral = 0
theta_average = 0

current = 0.0

#PID Constants
zero_offset = 0



#Time Keeping
millis = lambda: int(round(time.time() * 1000))
cur_time = millis()
start_time = cur_time
prev_time = cur_time
status_time = cur_time
motor_ack_timer = cur_time
lag_time = cur_time
motor_write_time = cur_time

max_fps = 0
avg_fps = 0
avg_fps_count = 0
lag_count = [0] * 5
lag_cutoff = 8


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
    counter_zz = 0
    while self.port.inWaiting() > 0 and counter_zz < 100:
      counter_zz += 1
      inByte = self.port.read(1)
      #print(self.port.inWaiting())
      #print(inByte)
      
      if not self.hasStart:
        if inByte == b'\x02':
          self.reset()
          self.hasStart = True
        else:
          print ("{} has noise ::{}".format( self.name, inByte ))
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
  global max_fps
  global avg_fps
  global avg_fps_count
  global lag_count 
  print( "-------------{}--------------".format( cur_time - start_time ) )
  print( "Motor Status: {} :: Set-> {} :: Theta -> {:.3g} :: Current -> {:.3g}".format( motor_status, motor_set_status, theta, current ) )
  print( "Solenoid Status: {} :: Set-> {}".format( sol_status, sol_set_status ) )
  print( "Loop Time::  Avg/{}: {:.3g} :: Max/{}{}: {}".format( avg_fps_count, avg_fps/avg_fps_count, lag_cutoff, lag_count, max_fps ))
  print( "IMU: {},{},{} :: RPM: {} :: Nunchuck: {}".format( imu_euler[0],imu_euler[1],imu_euler[2], motor_rpm, nun_x ))
  max_fps = 0
  avg_fps = 0
  avg_fps_count = 0
  lag_count = [0] * 5


def processMotorPacket(packet):
  global motor_status
  if not packet:
    print ("Error: Packet Empty")
  elif len(packet) == 0:
    print ("Error: Packet Empty")
  elif packet[0] == '0':
    print ("Emergency motor shutdown!!!")
    motor_status = 2
  elif packet[0] == 69:
    print ("Motor Error")
  #heartbeat #Todo need to check when the motor_status does not equal what rpi says it should be.
  elif packet[0] == 1 and len(packet)==2:
    if packet[1] == 0:
      motor_status = 2
    if packet[1] == 1:
      motor_status = 1
    if packet[1] == 2:
      motor_status = 0
  #motor acks
  elif packet[0] == 2 and len(packet)==2:
    if packet[1] == 0:
      print('Motor turned off')
      global motor_waiting_off_ack
      if motor_waiting_off_ack:
        motor_waiting_off_ack = False
        motor_status = 0
      else:
        print("Received ack wrongly motor_off")
    if packet[1] == 1:
      print('Motor turned on')
      global motor_waiting_on_ack
      if motor_waiting_on_ack:
        motor_waiting_on_ack = False
        motor_status = 1
        print( "Lag time :: {}".format( cur_time - lag_time ) )
      else:
        print("Received ack wrongly motor_on")

  elif packet[0] == 3 and len(packet) == 5:
    global motor_rpm
    motor_rpm = int.from_bytes(packet[1:5], byteorder='little', signed=True)
  else:
    print (packet)

def processImuPacket(packet):
  if not packet:
    print ("Error: Packet Empty")
  elif len(packet) == 0:
    print ("Error: Packet Empty")
  elif packet[0] == 69:
    print ("Imu Error")
  elif packet[0] == 1 and len(packet) == 73:
    global imu_euler
    global imu_gyro
    global imu_linaccel
    euler_x = struct.unpack('d', bytes(packet[1:9]))[0]
    euler_y = struct.unpack('d', bytes(packet[9:17]))[0]
    euler_z = struct.unpack('d', bytes(packet[17:25]))[0]
    imu_euler = (euler_x,euler_y,euler_z)
    gyro_x = struct.unpack('d', bytes(packet[25:33]))[0]
    gyro_y = struct.unpack('d', bytes(packet[33:41]))[0]
    gyro_z = struct.unpack('d', bytes(packet[41:49]))[0]
    imu_gyro = (gyro_x,gyro_y,gyro_z)
    linacc_x = struct.unpack('d', bytes(packet[49:57]))[0]
    linacc_y = struct.unpack('d', bytes(packet[57:65]))[0]
    linacc_z = struct.unpack('d', bytes(packet[65:73]))[0]
    imu_linaccel = (linacc_x,linacc_y,linacc_z)

  else:
    print (packet)

def processNunchuckPacket(packet):
  if not packet:
    print ("Nunchuck Error: Packet Empty")
  elif len(packet) != 3:
    print ("Nunchuck Error: Incorrect Packet Size")
  else:
    global nun_wheel
    global motor_set_status
    global nun_jump
    global sol_set_status
    global nun_x
    global nun_requires_press
    global rpi_nun_override
    global in_shutdown
    nun_wheel = packet[0]
    if not nun_requires_press: 
      if ( not rpi_nun_override and nun_wheel != motor_set_status ):
        motor_set_status = nun_wheel  
      nun_jump = packet[1]
      if ( not rpi_nun_override and nun_jump != sol_set_status ):
        sol_set_status = nun_jump  
      nun_x = int(packet[2])
    elif nun_wheel == 0: #Until we see a 0, we dont let the motor run
      nun_requires_press = False
      rpi_nun_override = False
    
def shutdown():
  if not in_shutdown:
    global motor_set_status
    motor_set_status = 0
    global in_shutdown
    in_shutdown = True
    global rpi_nun_override
    rpi_nun_override = True
    global nun_requires_press
    nun_requires_press = True
    print( "sent motor off packet" )
    s_motor.write(b'\x02')
    s_motor.write(b'\x02')
    s_motor.write(b'\x01')
    s_motor.write(b'\x00')

def resetPID():
  global theta_integral
  global theta_average
  global in_shutdown
  theta_integral = 0
  theta_average = 0
  in_shutdown = False


processImuSerial = ProcessSerial("imu", s_imu, processImuPacket)
processMotorSerial = ProcessSerial("motor", s_motor, processMotorPacket)
processSolenoidSerial = ProcessSerial("solenoid", s_solenoid, processMotorPacket)#todo process sol packet
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
while running:
  #Timers
  prev_time = cur_time
  cur_time = millis()
  frame_time = cur_time - prev_time
  max_fps = max(frame_time, max_fps)
  avg_fps += frame_time
  avg_fps_count += 1
  for i in range(5):    
    if ( frame_time > lag_cutoff + i ):
      lag_count[i] += 1
    
  #read from devices to update info
  for sr in SerialReaders:
    sr.readSerial()
  #use info to calculate what to return


    
  #make sure anything based on time is normalized to frame time.
  #calculate PID loop
  theta_prev = theta
  theta = imu_euler[1] - 1.1
  theta_deriv = (theta - theta_prev) / (frame_time + 1)
  #theta_integral = 
  theta_average = (frame_time/1000 * theta) + ((1-frame_time/1000) * theta_average)

  #Todo check signs(directions)
  current = 15.0 * theta + 33.0 * theta_deriv
  current = max(-MAX_CURRENT, min(current, MAX_CURRENT))

  #Todo Wind down(RPM Spindown)
  if ( abs(theta) > 20 or abs(motor_rpm) > 3000 ) :
    shutdown()
    1==1

  #write serial to devices
  if motor_connected:
    if ( motor_status == 0 and motor_set_status == 1 and not motor_waiting_on_ack):
      #Send Motor On Packet
      print( "sent motor on packet")
      s_motor.write(b'\x02')
      s_motor.write(b'\x02')
      s_motor.write(b'\x01')
      s_motor.write(b'\x01')
      motor_waiting_on_ack = True
      lag_time = cur_time
      resetPID()
      
    if ( motor_status == 1 and motor_set_status == 0 and not motor_waiting_off_ack):
      #Send Motor Off Packet
      print( "sent motor off packet" )
      s_motor.write(b'\x02')
      s_motor.write(b'\x02')
      s_motor.write(b'\x01')
      s_motor.write(b'\x00')
      motor_waiting_off_ack = True

    # Write motor current every loop if motor should be on.
    #motor_set_status == 1 and
    if (  cur_time - motor_write_time > 20 ):
      motor_write_time = cur_time
      s_motor.write(b'\x02')
      s_motor.write(b'\x05')
      s_motor.write(b'\x02')
      s_motor.write(bytearray(struct.pack('f', current)))
        
  if ( HasStatusUpdate and (cur_time - status_time) > StatusTime ):
    status_time = cur_time
    printStatus()

  time.sleep(.005)

#SHUTDOWN EVERYTHING

#Send Motor Off Packet
print( "sent motor shutdown packet" )
s_motor.write(b'\x02')
s_motor.write(b'\x01')
s_motor.write(b'\x00')

#Send Solenoid Off Packet







if imu_connected:
  s_imu.flush()
  s_imu.close()
if motor_connected:
  s_motor.flush()
  s_motor.close()
if sol_connected:
  s_solenoid.flush()
  s_solenoid.close()
if nunchuck_connected:
  s_nunchuck.flush()
  s_nunchuck.close()