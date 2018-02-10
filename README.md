# Project Palooza - Pogo
Arduino code for the IMU and Encoder.

The encoder we are using is the ["CUI AMT203-V"](https://www.digikey.com/product-detail/en/cui-inc/AMT203-V/102-2050-ND/2278846).

The IMU we are using is the ["9DOF Sensor Stick" version "SEN-10724"](https://www.sparkfun.com/products/retired/10724).
The code base currently in use is pulled from [here](https://github.com/Razor-AHRS/razor-9dof-ahrs/wiki/Tutorial#setting-up-the-hardware)

The IMU is not compatible with the Arduino Yun.
The IMU works with the Arduino Mega.


CS TODO:
  - Set up Raspberry Pi
    - Figure out how it interfaces with motors (I/O of both motor control and sensors)
    - Control system: RC vs Wifi etc
  - PID:
     - Constant time loop(? ask henry what that means later)
     - Figure out how to better reset data (especially Integral term)
    
   

  
