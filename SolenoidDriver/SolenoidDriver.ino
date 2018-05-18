#include "VescUart.h"
#include "datatypes.h"

// Pin assignments
const int SCOPE = A8;
const int SCOPE2 = A7;
const int POT = A9;
const int LED = 13;

#define INTERVAL    3000 // 150000 = 0.15 seconds
#define PERIOD      0.03 // s
#define MIDDLE      430
#define MAX_CURRENT 80.0f

#define OutputSerial false
#define MaxPacketSize 40

unsigned long time;
unsigned long prev_time;
unsigned long tx_time;
unsigned long count;

IntervalTimer myTimer;
volatile int currentPot;
int currentPotCopy;
volatile int pastPot;
volatile int derivative;
int derivativeCopy;
int pastDerivativeCopy;
int scope_current;


char inByte = 0;   // for incoming serial data

// Pin assignments
const int POT = A2;     // Linear potentiometer
const int CONTROL = A3; // Logic for gate driver

union DConv{
   double x;
   uint8_t c[sizeof (double)]; 
};
union DConv r_dconv;
union DConv w_dconv;


union FConv{
   float x;
   uint8_t c[sizeof (float)]; 
};
union FConv r_fconv;
union FConv w_fconv;


union IConv{
   int x;
   uint8_t c[sizeof (int)]; 
};
union IConv r_iconv;
union IConv w_iconv;


unsigned char serialBuffer[MaxPacketSize] = { 0 };
bool hasStart = false;
int bufferPos = 0;
int packetSize = -1;

int receivedPackets = 0;



void serialPrintDouble(double val){
  w_dconv.x = val;
  for (unsigned i = 0; i < sizeof (double); i++){
    Serial.write(w_dconv.c[i]);
  }
}

void serialPrintFloat(float val){
  w_fconv.x = val;
  for (unsigned i = 0; i < sizeof (float); i++){
    Serial.write(w_fconv.c[i]);
  }
}

void serialPrintInt(int val){
  w_iconv.x = val;
  for (unsigned i = 0; i < sizeof (int); i++){
    Serial.write(w_iconv.c[i]);
  }
}

double serialReadDouble(int bufferPos){
  for (unsigned i = 0; i < sizeof (double); i++) {
    r_dconv.c[i] = serialBuffer[bufferPos + i];
  }
  return r_dconv.x;
}

float serialReadFloat(int bufferPos){
  for (unsigned i = 0; i < sizeof (float); i++) {
    r_fconv.c[i] = serialBuffer[bufferPos + i];
  }
  return r_fconv.x;
}

int serialReadInt(int bufferPos){
  for (unsigned i = 0; i < sizeof (int); i++) {
    r_iconv.c[i] = serialBuffer[bufferPos + i];
  }
  return r_iconv.x;
}

void resetSerial(){
  hasStart = false;
  bufferPos = 0;
  packetSize = -1;
  
}


//Packets:
//0 -> Emergency Shutoff
//1 -> Jump
//2 -> Off
void processPacket(){
  if (serialBuffer[0] == '\0' && packetSize == 1)
  {
    // shutoff();
  }
  else if (serialBuffer[0] == '\1' && packetSize == 1)
  {
    // jumps at max
    analogWrite( SCOPE2, 200);

  }
  else if (serialBuffer[0] == '\2' && packetSize == 1)
  {
    // no jump
    analogWrite( SCOPE2, 50);
  }

  //always
  //potRead();
}

//from templateDriver
// void setup() {
//   Serial.begin( 9600 );
//   Serial.flush();
//   delay( 1000 );
// }


void setup() {
  Serial.begin(9600);
  // VESC UART
  Serial2.begin(9600);
  pinMode( POT, INPUT );
  pinMode( SCOPE, OUTPUT );
  pinMode( LED, OUTPUT);

  myTimer.begin(readPot, INTERVAL);  // blinkLED to run every 0.03 seconds
  analogWriteResolution( 8 );
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
}

struct bldcMeasure measuredValues;

//#define MAX_CURRENT 80.0f
float current = 0.0;
volatile bool on = true;
bool read_rpm = false;

void readPot() {
  currentPot = analogRead( POT );
  derivative = ( currentPot - pastPot ) / PERIOD;
  pastPot = currentPot;
}

// the loop function runs over and over again until power down or reset
void loop() {
  noInterrupts();
  derivativeCopy = derivative;
 // currentPotCopy = currentPot;
  interrupts();
/*
  derivativeCopy = map( derivativeCopy, -1000, 1000, 0, 255 );
  derivativeCopy = constrain( derivativeCopy, 0, 255 );
  //Serial.println( derivativeCopy);
  analogWrite( SCOPE, derivativeCopy );

  if ( pastDerivativeCopy != derivativeCopy ) {
    Serial.println( derivativeCopy );
  }
*/
  if ( currentPot < MIDDLE && derivativeCopy > 100 ){
    current = map( derivativeCopy, 100, 200, 0, MAX_CURRENT );
    current = constrain( current, 0, MAX_CURRENT);
  }
  else if ( currentPot > MIDDLE && derivativeCopy < 0 ) {
    current = map( derivativeCopy, 0, -200, 0, MAX_CURRENT );
    current = constrain( current, 0, MAX_CURRENT);
  }
  else current = 0.0;
  scope_current = map( current, 0, MAX_CURRENT, 50, 200 );
  scope_current = constrain( scope_current, 50, 200 );
  //Serial.println( derivativeCopy);
  //Serial.println( currentPot );
  analogWrite( SCOPE, scope_current );
  analogWrite( SCOPE2, scope_current );
  //VescUartSetCurrent( current );
  
  if ( current > 0 ) digitalWrite( LED, HIGH);
  else digitalWrite( LED, LOW );
  pastDerivativeCopy = derivativeCopy;

  //Read Serial for any updates from Master
  while ( Serial.available() > 0) {
    inByte = Serial.read();
    Serial.print(inByte);
    if (!hasStart){
      if ( inByte == '\2' ){

        resetSerial();
        hasStart = true;
      } else {
        //Serial.print("Has Noise");
        
      }
    } else if (bufferPos > MaxPacketSize-2)  {
      resetSerial();
    } 
    //second byte
    else if ( packetSize == -1 ) {
      packetSize = int(inByte);
    } 
    //rest of bytes
    else { 
      serialBuffer[bufferPos++] = inByte;
    }

    if ( packetSize != -1 && bufferPos == packetSize ){
      processPacket();
      receivedPackets++;
      resetSerial();
    }
  }


  //Write output back to master TODO: none right now
//   if(OutputSerial){
//     //First byte is always '\2'
//     Serial.print('\2');
//     //Second byte is the size of the rest of the packet. Double * 8 + Float * 4 + Int * 4 + Char * 1 
//     Serial.print('\x09');
//     Serial.print('\x01');
//   }
  
  delay(10);
}



//50 = off, 200