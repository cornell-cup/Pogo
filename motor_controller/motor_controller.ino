#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "VescUart.h"
#include "datatypes.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned long time;
unsigned long prev_time;
unsigned long tx_time;
unsigned long count;

void setup() {
  Serial.begin(9600);
  // VESC UART
  Serial1.begin(9600);
  Serial.println( "STARTING" );
  Serial.flush();
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  pinMode(3, INPUT_PULLUP);
  bno.setExtCrystalUse(true);

}

struct bldcMeasure measuredValues;

#define MAX_BUFFER_LEN 4
byte bindex = 0;
char buffer[MAX_BUFFER_LEN];

#define MAX_CURRENT 20.0f
#define CURRENT_ACCEL 0.20f
#define CURRENT_DECEL 0.20f
float current = 0.0;

float error = 0;
float theta = 0;
float theta_Prev = 0;
float theta_Speed = 0;
float theta_Speed_Prev = 0;
float theta_Integral = 0;
float theta_Integral_Max = 1;
float power = 0.0f; 
float pwrfactor = 10;


bool accel = true;
bool read_rpm = false;
bool isRunning = false;

// the loop function runs over and over again until power down or reset
void loop() {

  prev_time = time;
  time = millis();
  Serial.println( time - prev_time );
//  delay(5);

  if( !read_rpm ) {
    VescUartGetValue(measuredValues);
    tx_time = time;  
    read_rpm = true;
  }
  else {
    if( time - tx_time  > 120 ) {
      if( VescUartReadValue(measuredValues) ) {
//        Serial.print("Loop: "); Serial.println(count++);
//        SerialPrint(measuredValues);
        read_rpm = false;
      }
      else {
//        Serial.println("Failed to get data!");
        read_rpm = false;
      }
    }
  }

  sensors_event_t event;
  bno.getEvent(&event);
  error = (event.orientation.y);
  Serial.print("       ");
  Serial.println(error );
  error = error  + (0 * .2 * theta_Integral) -  ( .0007 * measuredValues.rpm);
  Serial.print("       ");
  Serial.println(error );

  if ( !isRunning ) {
    VescUartSetCurrentBrake(2.0f);
    if ( abs(error) < 0.5 ) {
      isRunning = true;
      theta_Integral = 0;
    }
  }
  else {
    if ( abs(error) > 12 || abs(measuredValues.rpm) > 2200 ) {
      isRunning = false;
      VescUartSetCurrentBrake(2.0f);
      theta_Integral = 0;
    }
    else {
      theta_Prev = theta;
      theta = error;
    
      theta_Speed_Prev = theta_Speed;
      theta_Speed = theta - theta_Prev;

      if(time-prev_time < 100){
        theta_Integral += (theta) * (time-prev_time) / 10000.0;
        theta_Integral = constrain(theta_Integral, -theta_Integral_Max, theta_Integral_Max);
      }
      Serial.print("               ");
      Serial.println( theta_Integral );
      
  float power;
  if (error>7) power = -(5.62 * error) - (33.0 * theta_Speed);
  else power = -(3.62 * error) - (33.0 * theta_Speed); 
  power = constrain(power, -MAX_CURRENT, MAX_CURRENT);
  VescUartSetCurrent(power);
  Serial.print("                      ");
  Serial.println( power );
  
    }    
  }
  
  delay(10);

//  while (Serial.available() > 0) {
//    int c = Serial.read();
//
//    if (c == '\n') {
//      bindex = 0;
//    }
//    else {
//      buffer[bindex] = (char) c;
//      bindex++;
//    }
//
//    if (bindex == MAX_BUFFER_LEN) {
//      // Process buffer
//      int value = (buffer[1] - '0') * 100 + (buffer[2] - '0') * 10 + (buffer[3] - '0');
//      if (buffer[0] == 'S') {
//        // Set motor current forward
//        float current = value / 100.f;
//        if (current < MAX_CURRENT) {
//          Serial.println("Set forward current");
//          VescUartSetCurrent(current);
//        }
//      }
//      else if (buffer[0] == 'R') {
//        // Set motor current backward
//        float current = value / 100.f;
//        if (current < MAX_CURRENT) {
//          Serial.println("Set backward current");
//          VescUartSetCurrent(-current);
//        }
//      }
//      else if (buffer[0] == 'B') {
//        // Set brake current
//        float current = value / 100.f;
//        if (current < MAX_CURRENT) {
//          Serial.println("Set brake current");
//          VescUartSetCurrentBrake(current);
//        }
//      }
//      bindex = 0;
//    }
//  }

}

