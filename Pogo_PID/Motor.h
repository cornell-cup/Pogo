/*
 * Motor.h
 * Header file for talking to the encoder
 */

#ifndef MOTOR
#define MOTOR

#define PWMPin 6

void motor_setup();
void write_power( int pwm );

#endif // MOTOR
