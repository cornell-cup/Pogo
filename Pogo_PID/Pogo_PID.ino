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
float preverror = 0;
float error = 0;
float deriv = 0;
float prevderiv = 0;
int awv = 0;

void setup() {
  motor_setup();
  encoder_setup();
  
  #ifdef DEBUG_PID
  Serial.begin(9600);
  Serial.print("STARTING");
  #endif // DEBUG_PID
}

void loop() {
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

  #ifdef DEBUG_ENCODER
  Serial.println(print_pos());
  #endif // DEBUG_ENCODER
}
