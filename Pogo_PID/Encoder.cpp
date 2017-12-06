/*
 * Encoder.cpp
 * .cpp file to read encoder values & debug from them
 */


#include <Arduino.h>
#include "Encoder.h"
#include <SPI.h>
// Default pins from library are
// MOSI = 11; MISO = 12; CLK = 13

#define DEBUG_ENCODER

// commands from datasheet
const byte NOP = 0x00; // NO oPeration (NOP)
const byte READ_POS = 0x10; // Read position

// vars for getting positions (12-bit, so need to separate into 2 holders)
uint16_t ABSposition = 0;
uint16_t ABSposition_last = 0;
uint8_t pos_array[2];
float pos_in_deg;
float deg = 0.00;

void encoder_setup()
{
  pinMode( CS, OUTPUT ); // Slave Select
  digitalWrite( CS, HIGH ); // deactivate SPI device
  SPI.begin();
  SPI.setBitOrder( MSBFIRST );
  SPI.setDataMode( SPI_MODE0 );
  SPI.setClockDivider( SPI_CLOCK_DIV32 ); // sets SPI clock to 1/32 freq of sys CLK (ie, 16/32 MHz)

#ifdef DEBUG_ENCODER
  Serial.begin( 9600 );
  Serial.println( "STARTING" );
  Serial.flush();
  delay( 2000 );
#endif // DEBUG_ENCODER

  SPI.end();

}
uint8_t SPI_T ( uint8_t msg )    // Repetive SPI transmit sequence
{
  digitalWrite( CS, LOW );     // active SPI device
  uint8_t msg_temp = SPI.transfer( msg );    // send and recieve
  digitalWrite( CS, HIGH );    // deselect SPI device
  return ( msg_temp );     // return recieved byte
}

float get_pos () {
  pos_array[0] &= ~ 0xF0; // mask out first 4 bits (12-bit position) from two 8 bit #s
  ABSposition = pos_array[0] << 8;    // shift MSB to correct ABSposition in ABSposition message
  ABSposition += pos_array[1];    // add LSB to ABSposition message to complete message

  if ( true /* ABSposition != ABSposition_last */ ) // if nothing has changed dont waste time sending position
  {
    ABSposition_last = ABSposition;    // set last position to current position
    deg = ABSposition;
    deg = deg * 0.08789;    // aprox 360/4096, 4096 because position is 12-bit (2^12 = 4096)
    deg = deg - 202.3;
    if ( deg < 0 ) deg = deg + 360;
    else deg = -200.0;
  }

   return ( deg );     // send position in degrees
}

void print_pos () {
  Serial.println( get_pos () );
}

float encoder_pos()
{
  // To read pos, datasheet lays out steps
  // 1. Master ends READ_POS. Encoder responds with an idle character
  // 2. Continue sending NO_OPERATION while encoder response is 0xA5
  // 3. If response is 0x10 (READ_POS), send NO_OPERATION & get MSB pos (lower 4 are upper 4 of 12 bit pos)
  // 4. Send 2nd NO_OPERATION cmd & get LSB pos (lower 8 bits of 12 bit pos)

  uint8_t recieved = 0xA5;    //just a temp variable
  ABSposition = 0;    //reset position variable

  SPI.begin();    // start transmission
  digitalWrite( CS, LOW ); // active up SPI device

  SPI_T( READ_POS );   // issue read command

  recieved = SPI_T( NOP );    // issue NOP to check if encoder is ready to send

  while ( recieved != READ_POS )    // loop while encoder is not ready to send
  {
    recieved = SPI_T( NOP );    // check again if encoder is still working
    delayMicroseconds(20);    // wait a bit (2 milliseconds)
  }

  pos_array[0] = SPI_T( READ_POS );    // recieve MSB
  pos_array[1] = SPI_T( READ_POS );    // recieve LSB

  digitalWrite( CS, HIGH );  // make sure SPI is deactivated
  SPI.end();    // end transmition

  pos_in_deg = get_pos ();
  delayMicroseconds(20);    // datasheet says 20us b/w reads is good
  
  return pos_in_deg;
}

