/*
 * Encoder.h
 * Header file for talking to the encoder
 */

#ifndef ENCODER
#define ENCODER

const int CS = 3; // Chip select

void encoder_setup();
uint8_t SPI_T ( uint8_t msg );
uint16_t get_pos ( uint8_t position_array[] );
float encoder_pos();

void print_pos ( uint8_t position_array[] ); // if-def'ed

#endif // ENCODER
