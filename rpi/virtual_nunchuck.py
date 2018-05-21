import serial
import time


#Run socat -d -d pty,raw,echo=0 pty,raw,echo=0 
#to start the ports
#Run three terminals
serial_read = serial.Serial('/dev/pts/2')
serial_write = serial.Serial('/dev/pts/4')
while 1:

    i = input("1: All off, 2: Wheel On, 3: Jump On, < > move :")

    if i == '1':
        print( "All off" )
        serial_write.write(b'\x02')
        serial_write.write(b'\x03')
        serial_write.write(b'\x00')
        serial_write.write(b'\x00')
        serial_write.write(b'\x83')
        
    elif i == '2':
        print ("Wheel On")
        serial_write.write(b'\x02')
        serial_write.write(b'\x03')
        serial_write.write(b'\x01')
        serial_write.write(b'\x00')
        serial_write.write(b'\x83')
        
    elif i == '3':
        print( "Jump On")
        serial_write.write(b'\x02')
        serial_write.write(b'\x03')
        serial_write.write(b'\x01')
        serial_write.write(b'\x01')
        serial_write.write(b'\x83')
    #elif i == '<':

    #elif i == '>':

    time.sleep(.1)
