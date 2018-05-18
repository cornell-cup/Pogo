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
  //Serial.print( "Derivative " ); Serial.println( derivativeCopy);
  //Serial.print( "Potentiometer " ); Serial.println( currentPot );
  //Serial.print( "SCOPE " );Serial.println( scope_current);
  analogWrite( SCOPE, scope_current );
  analogWrite( SCOPE2, scope_current );
  //VescUartSetCurrent( current );
  
  if ( current > 0 ) digitalWrite( LED, HIGH);
  else digitalWrite( LED, LOW );
  pastDerivativeCopy = derivativeCopy;
}


