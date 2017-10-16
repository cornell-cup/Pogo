/*
 * Motor.cpp
 * .cpp file to contol the motor
 */

#include <Arduino.h>
#include "Motor.h"
const int PWMPin = 6; // Motor control pin to Pin 6

int val = 230; // variable to store the read value; range is 26-230
int valmin = 100;

short i = 0;
short j = 0;
float pwrfactor = 10;

void motor_setup() {
  pinMode(PWMPin, OUTPUT);
}

