//int EN = 6;
int PWMPin = 6;      // LED connected to digital pin 9
int DIRPin = 7;
int val = 230;         // variable to store the read value; range is 26-230
int valmin = 100;
#include <SPI.h>
// Default pins from library are
// MOSI = 11; MISO = 12; CLK = 13
const int CS = 3; // Chip select

short i = 0;
short j = 0;
float pwrfactor = 10;

float theta = 180;
float theta_Prev = 180;
float theta_Speed = 0;
float theta_Speed_Prev = 0;

float power = 0;
float oldpower = 0; 


//vars for PID
float preverror = 0;
float error = 0;
float deriv = 0;
float prevderiv = 0;

// commands from datasheet
const byte NOP = 0x00; // NO oPeration (NOP)
const byte READ_POS = 0x10; // Read position

// vars for getting positions (12-bit, so need to separate into 2 holders)
uint16_t ABSposition = 0;
uint16_t ABSposition_last = 0;
uint8_t pos_array[2];
float deg = 0.00;


void setup()

{

  
  pinMode( CS, OUTPUT ); // Slave Select
  digitalWrite( CS, HIGH ); // deactivate SPI device
  digitalWrite(DIRPin, LOW);
  SPI.begin();
  SPI.setBitOrder( MSBFIRST );
  SPI.setDataMode( SPI_MODE0 );
  SPI.setClockDivider( SPI_CLOCK_DIV32 ); // sets SPI clock to 1/32 freq of sys CLK (ie, 16/32 MHz)
  Serial.begin( 9600 );
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
   return( msg_temp );      // return recieved byte
}

float print_pos ( uint8_t position_array[] ) {
  position_array[0] &=~ 0xF0; // mask out first 4 bits (12-bit position) from two 8 bit #s
  ABSposition = pos_array[0] << 8;    // shift MSB to correct ABSposition in ABSposition message
  ABSposition += pos_array[1];    // add LSB to ABSposition message to complete message
    
  if ( true/*ABSposition != ABSposition_last && ( abs( ABSposition * 0.08789 - ABSposition_last * 0.08789 ) < 10 )*/ ) // if nothing has changed dont waste time sending position
  {
    ABSposition_last = ABSposition;    // set last position to current position
    deg = ABSposition;
    deg = deg * 0.08789;    // aprox 360/4096, 4096 because position is 12-bit (2^12 = 4096)
    deg = deg - 201.8;
    if ( deg < 0) { deg = deg + 360; } 
    //Serial.println(deg);     // send position in degrees
  }  
  else {
    deg = -1.0; 
  }

  pinMode(PWMPin, OUTPUT);   // sets the pin as output
  return(deg);
//  pinMode(EN, OUTPUT);
//  digitalWrite(EN, HIGH);
//  analogWrite(PWMPin, val);  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255


}

void write_power ( int pwm ) {
  if ( pwm < 26 && pwm >= 0){
      pwm = 26;
  }
  else if (pwm > -26 && pwm < 0 ) {
      pwm = -26;
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



void loop()

{ 
  // To read pos, datasheet lays out steps
  // 1. Master ends READ_POS. Encoder responds with an idle character
  // 2. Continue sending NO_OPERATION while encoder response is 0xA5
  // 3. If response is 0x10 (READ_POS), send NO_OPERATION & get MSB pos (lower 4 are upper 4 of 12 bit pos)
  // 4. Send 2nd NO_OPERATION cmd & get LSB pos (lower 8 bits of 12 bit pos)
  
   uint8_t recieved = 0xA5;    //just a temp variable
   ABSposition = 0;    //reset position variable
   
   SPI.begin();    // start transmition
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

   
   print_pos ( pos_array );
   

   if( deg > 0  && deg > 100 && deg < 260 && abs(deg-theta) < 4){

    theta_Prev = theta;
    theta = deg;

    theta_Speed_Prev = theta_Speed;
    theta_Speed = theta - theta_Prev;

//    Serial.println(theta_Speed);
    
    error = deg - 180;
    if ( error < 0) {
      error + 360;
    }

    float multiple = .45;
    if( error < 0) {
      multiple = .45;
    }
    float pid = -(multiple * error) - (5.7 * theta_Speed) + (.005 * power);
    if(abs(pid) < 150){    
      power = power +  pid;
      if ( power > 230) {
        power = 230;
      }
      if ( power < -230) {
        power = -230;
      }

      Serial.println(power);

      write_power(round(power));
      oldpower = power;   
    }
    
    
   } 
    
   delayMicroseconds(20);    // datasheet says 20us b/w reads is good

}
