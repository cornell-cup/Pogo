/*
 * Encoder.h
 * Header file for talking to the encoder
 */

#ifndef ENCODER
#define ENCODER

const int CS = 3; // Chip select

void encoder_setup();
uint8_t SPI_T ( uint8_t msg );
float get_pos ();
float encoder_pos();

void print_pos (); // if-def'ed

#endif // ENCODER
