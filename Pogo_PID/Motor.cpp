/*
 * Motor.cpp
 * .cpp file to contol the motor
 */

#include <Arduino.h>
#include "Motor.h"
//const int PWMPin = 6; // Motor control pin to Pin 6
const int DIRPin = 7; // TODO ask Henry about this

int val = 230; // variable to store the read value; range is 26-230
int valmin = 100;

short i = 0;
short j = 0;
float pwrfactor = 10;

void motor_setup() {
  pinMode(PWMPin, OUTPUT);
  digitalWrite(DIRPin, LOW);
}

void write_power( int pwm ){
  if ( pwm < 26 && pwm >= 0 ) {
    pwm = 26;
  }
  else if ( pwm < -26 && pwm < 0 ) {
    pwm = -26;
  }
  // Serial.println( pwm );
  if ( pwm > 0 ) {
    analogWrite( PWMPin, pwm );
    digitalWrite( DIRPin, LOW );
  }
  else {
    analogWrite( PWMPin, -pwm );
    digitalWrite( DIRPin, HIGH );
  }
}

