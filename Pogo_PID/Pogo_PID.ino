/*
 * Pogo_PID.ino
 * .ino file for PID control
 * 1. Uses values from the encoder ("Encoder.cpp")
 * 2. Controls the motor ("Motor.cpp")
 */

#include <Arduino.h>
#include "Encoder.h"
#include "Motor.h"

#define DEBUG_PID

// vars for PID
/*
float preverror = 0;
float error = 0;
float deriv = 0;
float prevderiv = 0;
int awv = 0;
*/
float power = 0;

float theta = 0;
float theta_Prev = 0;
float theta_Speed = 0;
float theta_Speed_Prev = 0;

float error = 0;
float imax = 300;
float iarr[300];
int iptr = 0;
float integral = 0;

void setup() {
  motor_setup();
  encoder_setup();
  
  #ifdef DEBUG_PID
  Serial.begin(9600);
  Serial.print("STARTING");
  #endif // DEBUG_PID
}

void loop() {
  /*
  if ( encoder_pos() > 0 ) {
    error = encoder_pos() - 180;
  }
  if ( error < 0 ) {
    error + 360;
    awv = 230;
  } 
  else {
    awv = 90;
  }
  // awv = round( 180 - ( abs( error ) * error * 20 ) );
  if ( awv >= 26 && awv <= 230 ) {
    analogWrite( PWMPin, awv );
  }
  else {
    analogWrite( PWMPin, 26 );
  }
*/
  print_pos();
  error = get_pos() - 180;

  if ( error > -80 && error < 80 && abs(error - theta) < 4 ) {
    theta_Prev = theta;
    theta = error;

    theta_Speed_Prev = theta_Speed;
    theta_Speed = theta - theta_Prev;

    float amp = abs(error) * 1.404;
    integral += amp;
    iptr += 1;
    if ( iptr >= imax ) {
      iptr = 0;
    }
    integral -= iarr[iptr];
    iarr[iptr] = amp;
    Serial.println(amp);

    float pid = -(0.45 * error) - (5.7* theta_Speed) + (.005 * (power));
    power = power +  pid;
    power = constrain(power, -230, 230);
    write_power(round(power));
  }

  #ifdef DEBUG_ENCODER
  Serial.println(print_pos());
  #endif // DEBUG_ENCODER
}
