#include "VescUart.h"
#include "datatypes.h"

const int SCOPE = A8;
const int POT = A9;
const int LED = 13;

#define INTERVAL 3000 // 150000 = 0.15 seconds
#define PERIOD   0.03 // s
#define MIDDLE 430
#define MAX_CURRENT 80

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

#define MAX_CURRENT 80.0f
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
  currentPotCopy = currentPot;
  interrupts();
/*
  derivativeCopy = map( derivativeCopy, -1000, 1000, 0, 255 );
  derivativeCopy = constrain( derivativeCopy, 0, 255 );
  //Serial.println( derivativeCopy);
  analogWrite( SCOPE, derivativeCopy );
*/
  if ( currentPot < MIDDLE && derivativeCopy > 0 ){
    current = map( derivativeCopy, 0, 100, 0, MAX_CURRENT );
    current = constrain( current, 0, MAX_CURRENT);
  }
  else if ( currentPot > MIDDLE && derivativeCopy < 0 ) {
    current = map( derivativeCopy, 0, -100, 0, MAX_CURRENT );
    current = constrain( current, 0, MAX_CURRENT);
  }
  else current = 0.0;
  scope_current = map( current, 0, MAX_CURRENT, 0, 255 );
  scope_current = constrain( scope_current, 0, 255 );
  //Serial.println( derivativeCopy);
  //Serial.println( currentPot );
  analogWrite( SCOPE, scope_current );
  VescUartSetCurrent( current );
  
  if ( current > 0 ) digitalWrite( LED, HIGH);
  else digitalWrite( LED, LOW );

  /*
  time = millis();
//  Serial.println( time - prev_time );
  if (on && time - prev_time  > 160){
      VescUartSetCurrent(0.0);
      on = false;
      prev_time = time;
      //Serial.println(0.0);
  }
  else if( !on && time - prev_time > 160) {
    VescUartSetCurrent(MAX_CURRENT);
    on = true;
    prev_time = time;
    //Serial.println(80.0);
    
  }*/
}

