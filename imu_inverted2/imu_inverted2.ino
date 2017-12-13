#include <SPI.h>
// Default pins from library are
// MOSI = 11; MISO = 12; CLK = 13
const int CS = 3; // Chip select

int PWMPin = 6;      // LED connected to digital pin 9
int DIRPin = 7;
int ENPin = 8;
int val = 230;         // variable to store the read value; range is 26-230
int valmin = 100;
// Default pins from library are
// MOSI = 11; MISO = 12; CLK = 13

float theta = 0;
float theta_Prev = 0;
float theta_Speed = 0;
float theta_Speed_Prev = 0;

float power = 100;

unsigned long time;
unsigned long prev_time;

//vars for PID
float error = 0;
float error_enc = 0;

// commands from datasheet
const byte NOP = 0x00; // NO oPeration (NOP)
const byte READ_POS = 0x10; // Read position

// vars for getting positions (12-bit, so need to separate into 2 holders)
uint16_t ABSposition = 0;
uint16_t ABSposition_last = 0;
uint8_t pos_array[2];
float deg = 0.00;


/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// HARDWARE OPTIONS
/*****************************************************************/
// Select your hardware here by uncommenting one line!
//#define HW__VERSION_CODE 10125 // SparkFun "9DOF Razor IMU" version "SEN-10125" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10736 // SparkFun "9DOF Razor IMU" version "SEN-10736" (HMC5883L magnetometer)
//#define HW__VERSION_CODE 10183 // SparkFun "9DOF Sensor Stick" version "SEN-10183" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10321 // SparkFun "9DOF Sensor Stick" version "SEN-10321" (HMC5843 magnetometer)
#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)


// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT__BAUD_RATE 57600

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 20  // in milliseconds

// Output mode definitions (do not change)
#define OUTPUT__MODE_CALIBRATE_SENSORS 0 // Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES 1 // Outputs yaw/pitch/roll in degrees
#define OUTPUT__MODE_SENSORS_CALIB 2 // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 3 // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 4 // Outputs calibrated AND raw sensor values for all 9 axes
// Output format definitions (do not change)
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float

// Select your startup output mode and format here!
int output_mode = 1;
int output_format = OUTPUT__FORMAT_TEXT;

// Select if serial continuous streaming output is enabled per default on startup.
#define OUTPUT__STARTUP_STREAM_ON false  // true or false

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
boolean output_errors = false;  // true or false

#define OUTPUT__HAS_RN_BLUETOOTH false  // true or false


// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = -270.0/250.0  -249.0/267.0  -282.0/226.0"
#define ACCEL_X_MIN ((float) -270)
#define ACCEL_X_MAX ((float) 250)
#define ACCEL_Y_MIN ((float) -249)
#define ACCEL_Y_MAX ((float) 267)
#define ACCEL_Z_MIN ((float) -282)
#define ACCEL_Z_MAX ((float) 226)

// Magnetometer (standard calibration mode)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN ((float) -600)
#define MAGN_X_MAX ((float) 600)
#define MAGN_Y_MIN ((float) -600)
#define MAGN_Y_MAX ((float) 600)
#define MAGN_Z_MIN ((float) -600)
#define MAGN_Z_MAX ((float) 600)

// Magnetometer (extended calibration mode)
// Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
#define CALIBRATION__MAGN_USE_EXTENDED true
const float magn_ellipsoid_center[3] = {56.1895, 120.936, 102.475};
const float magn_ellipsoid_transform[3][3] = {{0.00000, 0.00000, 0.00000}, {0.00000, 0.00000, 0.00000}, {0.00000, 0.00000, 0.00000}};

// Gyroscope
// "gyro x,y,z (current/average) = .../-463.13  .../291.66  .../-156.4"
#define GYRO_AVERAGE_OFFSET_X ((float) -463.13)
#define GYRO_AVERAGE_OFFSET_Y ((float) 291.66)
#define GYRO_AVERAGE_OFFSET_Z ((float) -156.40)



// DEBUG OPTIONS
/*****************************************************************/
// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false
// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME false


/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/










// Check if hardware version code is defined
#ifndef HW__VERSION_CODE
// Generate compile error
#error YOU HAVE TO SELECT THE HARDWARE YOU ARE USING! See "HARDWARE OPTIONS" in "USER SETUP AREA" at top of Razor_AHRS.ino!
#endif

#include <Wire.h>

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))


// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

// Stuff
#define STATUS_LED_PIN 13  // Pin number of status LED
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];

float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;

// DCM variables
float MAG_Heading;
float Accel_Vector[3] = {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3] = {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3] = {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3] = {0, 0, 0}; // Omega Proportional correction
float Omega_I[3] = {0, 0, 0}; // Omega Integrator
float Omega[3] = {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles
float yaw;
float pitch;
float roll;

// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
boolean output_stream_on;
boolean output_single_on;
int curr_calibration_sensor = 2;
boolean reset_calibration_session_flag = true;
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

void read_sensors() {
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion() {
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  read_sensors();
  timestamp = millis();

  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

  // GET ROLL
  // Compensate pitch of gravity vector
  Vector_Cross_Product(temp1, accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = atan2(temp2[1], temp2[2]);

  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;

  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
  // Compensate accelerometer error
  accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
  accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
  accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

  // Compensate magnetometer error
#if CALIBRATION__MAGN_USE_EXTENDED == true
  for (int i = 0; i < 3; i++)
    magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
  Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
#else
  magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
  magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
  magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif

  // Compensate gyroscope error
  gyro[0] -= GYRO_AVERAGE_OFFSET_X;
  gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
  gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}

// Reset calibration session if reset_calibration_session_flag is set
void check_reset_calibration_session()
{
  // Raw sensor values have to be read already, but no error compensation applied

  // Reset this calibration session?
  if (!reset_calibration_session_flag) return;

  // Reset acc and mag calibration variables
  for (int i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = accel[i];
    magnetom_min[i] = magnetom_max[i] = magnetom[i];
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;

  reset_calibration_session_flag = false;
}

void turn_output_stream_on()
{
  output_stream_on = true;
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void turn_output_stream_off()
{
  output_stream_on = false;
  digitalWrite(STATUS_LED_PIN, LOW);
}

// Blocks until another byte is available on serial port
char readChar()
{
  while (Serial.available() < 1) { } // Block
  return Serial.read();
}

void write_power ( int pwm ) {
  if ( pwm < 27 && pwm >= 0) {
    pwm = 27;
  }
  else if (pwm > -27 && pwm < 0 ) {
    pwm = -27;
  }
  //  Serial.println(pwm);
  if ( pwm > 0) {
    analogWrite(PWMPin, pwm);
    digitalWrite(DIRPin, LOW);

  } else {
    analogWrite(PWMPin, -pwm);
    digitalWrite(DIRPin, HIGH);
  }
}

void setup()
{
  // Init serial output
  Serial.begin(OUTPUT__BAUD_RATE);

  // Init status LED
  pinMode (STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Init sensors
  delay(50);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();

  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();

  // Init output
#if (OUTPUT__HAS_RN_BLUETOOTH == true) || (OUTPUT__STARTUP_STREAM_ON == false)
  turn_output_stream_off();
#else
  turn_output_stream_on();
#endif
  //  digitalWrite(DIRPin, LOW);
  pinMode(PWMPin, OUTPUT);   // sets the pin as output
  pinMode(DIRPin, OUTPUT);   // sets the pin as output
  pinMode(ENPin, OUTPUT); // sets the pin as output
  digitalWrite(ENPin, HIGH);
  analogWrite(PWMPin, 100);
  digitalWrite(DIRPin, LOW);

  pinMode( CS, OUTPUT ); // Slave Select
  digitalWrite( CS, HIGH ); // deactivate SPI device
  SPI.begin();
  SPI.setBitOrder( MSBFIRST );
  SPI.setDataMode( SPI_MODE0 );
  SPI.setClockDivider( SPI_CLOCK_DIV32 ); // sets SPI clock to 1/32 freq of sys CLK (ie, 16/32 MHz)
  Serial.println( "STARTING" );
  Serial.flush();
  delay( 2000 );
  SPI.end();

}

uint8_t SPI_T ( uint8_t msg )    // Repetive SPI transmit sequence
{
  digitalWrite( CS, LOW );     // active SPI device
  uint8_t msg_temp = SPI.transfer( msg );    // send and recieve
  digitalWrite( CS, HIGH );    // deselect SPI device
  return ( msg_temp );     // return recieved byte
}

float print_pos ( uint8_t position_array[] ) {
  position_array[0] &= ~ 0xF0; // mask out first 4 bits (12-bit position) from two 8 bit #s
  ABSposition = pos_array[0] << 8;    // shift MSB to correct ABSposition in ABSposition message
  ABSposition += pos_array[1];    // add LSB to ABSposition message to complete message

  if ( true/*ABSposition != ABSposition_last && ( abs( ABSposition * 0.08789 - ABSposition_last * 0.08789 ) < 10 )*/ ) // if nothing has changed dont waste time sending position
  {
    ABSposition_last = ABSposition;    // set last position to current position
    deg = ABSposition;
    deg = deg * 0.08789;    // aprox 360/4096, 4096 because position is 12-bit (2^12 = 4096)
    deg = deg - 271.2;
    if ( deg < 0) {
      deg = deg + 360;
    }
  }
  else {
    deg = -200.0;
  }

  pinMode(PWMPin, OUTPUT);   // sets the pin as output
  return (deg);
  //  pinMode(EN, OUTPUT);
  //  digitalWrite(EN, HIGH);
  //  analogWrite(PWMPin, val);  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255


}

// Main loop
void loop()
{

//    Serial.print("loop time (ms) = ");
//    Serial.println(millis() - timestamp);
  // Time to read the sensors again?
  if ((millis() - timestamp) >= OUTPUT__DATA_INTERVAL)
  {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else G_Dt = 0;

    // Update sensor readings
    read_sensors();
    
    // Apply sensor calibration
    compensate_sensor_errors();

    // Run DCM algorithm
    Compass_Heading(); // Calculate magnetic heading
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();




//    uint8_t recieved = 0xA5;    //just a temp variable
//    ABSposition = 0;    //reset position 

//    SPI.begin();    // start transmition
//    digitalWrite( CS, LOW ); // active up SPI device
//
//    SPI_T( READ_POS );   // issue read command
//
//    recieved = SPI_T( NOP );    // issue NOP to check if encoder is ready to send
//    int c = 0;
//    while ( recieved != READ_POS  && c < 200)    // loop while encoder is not ready to send
//    {
//      recieved = SPI_T( NOP );    // check again if encoder is still working
//      delayMicroseconds(20);    // wait a bit (2 milliseconds)
//      c+= 1;
//    }
//
//    pos_array[0] = SPI_T( READ_POS );    // recieve MSB
//    pos_array[1] = SPI_T( READ_POS );    // recieve LSB
//
//    digitalWrite( CS, HIGH );  // make sure SPI is deactivated
//    SPI.end();    // end transmition


//    print_pos ( pos_array );
//    error_enc  = deg - 180;
//    Serial.println(error_enc);

//    Serial.print("Time: ");
//    prev_time = time;
//    time = millis();
//
//    //prints time since program started
//    Serial.println(time - prev_time);


    error = -TO_DEG(pitch) - 1;
    Serial.println(error);
//  Serial.println(error_enc - error);

    

//    if (error > -80 && error < 80 && abs(error - theta) < 20) {
//
//      theta_Prev = theta;
//      theta = error;
//
//      theta_Speed_Prev = theta_Speed;
//      theta_Speed = theta - theta_Prev;
//
//      //      Serial.println(power);
//
//
//      float pid = -(4 * error) - (4 * theta_Speed) - (0.005 * (power));
//      power = power +  pid;
//      power = constrain(power, -230, 230);
//      write_power(round(power));
//
//    }
  }


}
