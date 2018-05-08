#include "VescUart.h"
#include "datatypes.h"

#define OutputSerial false
#define MaxPacketSize 40


struct bldcMeasure measuredValues;

unsigned long time;
unsigned long prev_time;
unsigned long tx_time;
unsigned long count;

bool read_rpm = false;
bool isRunning = false;

float error = 0.0;

#define MAX_CURRENT 20.0f
#define CURRENT_ACCEL 0.05f
#define CURRENT_DECEL 0.20f
float current = 0.0;
bool accel = true;


char inByte = 0;   // for incoming serial data

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
  serialBuffer[0] = 0;
  
}

void processPacket(){
  //Use serialBuffer
  //TODO for each;
  if (serialBuffer[0] == '\1' && packetSize == 5){
    error = serialReadFloat(1);    
  }
}

void setup() {
  //Serial is to the rpi
  Serial.begin( 115200 );
  Serial.flush();
  //Serial1 is to the motor controller
  Serial1.begin( 9600 );
  delay( 1000 );
}


void loop() {
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
        ;
      }
    } else if (bufferPos > MaxPacketSize-2)  {
      resetSerial();
    } else if ( packetSize == -1 ) {
      packetSize = int(inByte);
    } else { 
      serialBuffer[bufferPos++] = inByte;
    }

    if ( packetSize != -1 && bufferPos == packetSize ){
      processPacket();
      receivedPackets++;
      resetSerial();
    }
  }


  /*
   * Possible Values
  Loop: 137
  avgMotorCurrent: 0.00
  avgInputCurrent: 0.00
  dutyCycleNow: -0.00
  rpm: 0
  inputVoltage: 23.90
  ampHours: 0.00
  ampHoursCharges: 0.00
  tachometer: -2
  tachometerAbs: 2  
  */

  prev_time = time;
  time = millis();
  //Serial.println( time - prev_time );

  
  if( !read_rpm ) {
    //We send a request to the motor controller to send us back data.
    //VescUartGetValue(measuredValues);
    tx_time = time;  
    read_rpm = true;
  }
  else {
    //We need to wait for the packet to come back before we read it.
    if( false &&  time - tx_time  > 160 ) {
      if( VescUartReadValue(measuredValues) ) {
        Serial.print("Loop: "); Serial.println(count++);
        SerialPrint(measuredValues);
        read_rpm = false;
      }
      else {
        Serial.print('E');
        read_rpm = false;
      }
    }
  }

  if (accel) {
    if (current < MAX_CURRENT - 0.5f) {
      current += CURRENT_ACCEL;
    }
    else {
      accel = false;
    }
    VescUartSetCurrent(current);
  }
  else {
    if (current > -MAX_CURRENT + 0.5f) {
      current -= CURRENT_ACCEL;
    }
    else {
      accel = true;
    }
    VescUartSetCurrent(current);

  }
  Serial.println(current);



  //Write output back to master
  if(OutputSerial){
    //First byte is always '\2'
    Serial.print('\2');
    //Second byte is the size of the rest of the packet. Double * 8 + Float * 4 + Int * 4 + Char * 1 
    Serial.print('\x09');
    Serial.print('\x01');
  }
  
//  delay(10);/
}

