#include "VescUart.h"
#include "datatypes.h"

#define OutputSerial false
#define MaxPacketSize 40
#define HasHeartbeat true
#define HeartbeatTime 100
#define ShutoffTime 100
#define ReadMotorControllerTime 160
#define BreakCurrent 4.0

struct bldcMeasure measuredValues;

unsigned long time;
unsigned long prev_time;
unsigned long tx_time;
unsigned long heartbeat_time;
unsigned long last_received;
unsigned long count;

//Used to determine if you are waiting for a rpm packet to come back from the motor controller.
bool read_rpm = false;
//Our motor should be off until it receives an input from the rpi.
bool motor_on = false;
bool m_on_ack = false;
bool m_off_ack = false;
bool emergency_shutoff = false;


float current = 0.0;

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

void resetAll(){
  resetSerial();
  time = millis();
  prev_time = time;
  tx_time = time;
  heartbeat_time = time;
  last_received = time;
  count = 0;

  read_rpm = false;
  motor_on = false;
  m_on_ack = false;
  m_off_ack = false;
  emergency_shutoff = false;
  current = 0.0;
  inByte = 0;   // for incoming serial data
  
  VescUartSetCurrentBrake(BreakCurrent);

}

void shutoff(){
  emergency_shutoff = true;
  VescUartSetCurrentBrake(BreakCurrent);
  current = 0;
  motor_on = false;
  Serial.print('\2');         //Start Packet
  Serial.print('\x01');       //One Bytes
  Serial.print('\x00');       //Packet Id:0
}

//Packets:
//0 -> Emergency Shutoff
//1 -> Motor on/off
//2 -> Current Control
void processPacket(){
  if (serialBuffer[0] == '\0' && packetSize == 1){
    shutoff();
  } else if (serialBuffer[0] == '\1' && packetSize == 2){
    bool com = serialBuffer[1] == '\1';
    if ( com != motor_on ) {
      motor_on = com;
      if ( motor_on ) { 
        m_off_ack = false;      
      } else {
        m_on_ack = false;
      }
    }
  } else if (serialBuffer[0] == '\2' && packetSize == 5){
    current = serialReadFloat(1);    
  }
  last_received = time;
}

void setup() {
  //Serial is to the rpi
  Serial.begin( 115200 );
  Serial.flush();
  //Serial1 is to the motor controller
  Serial1.begin( 9600 );
  delay( 100 );
}


void loop() {
  prev_time = time;
  time = millis();
  //Serial.println( time - prev_time ); //Used to print loop frequency.
  
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

  if( !read_rpm ) {
    //We send a request to the motor controller to send us back data.
    //VescUartGetValue(measuredValues);
    tx_time = time;  
    read_rpm = true;
  }
  else {
    //We need to wait for the packet to come back before we read it.
    if( false &&  time - tx_time  > ReadMotorControllerTime ) {
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

  //If we have not received an update from the rpi in 100ms, we shut the motor off.
  if( motor_on && time - last_received > ShutoffTime ) {
    shutoff();
  }

  //Actually control the motor current
  if( !motor_on ) {
    current = 0.0;
  }
  if( !emergency_shutoff ) {
    VescUartSetCurrent(current);    
  } else {
    VescUartSetCurrentBrake(BreakCurrent);
  }
  
  //Write output back to master
  if(OutputSerial){
    
    //First byte is always '\2'
    //Second byte is the size of the rest of the packet. Double * 8 + Float * 4 + Int * 4 + Char * 1 
    //Third byte is the packet id.
    //Rest of the bytes is payload.

    //Packets Out
    //0 -> Emergency Shutoff
    //1 -> Heartbeat
    //2 -> Command Ack

    //We output a heartbeat so that the rpi knows our status.
    //If no signal is detected within 200 ms, we know that the arduino is unresponsive.
    if( HasHeartbeat && time - heartbeat_time > HeartbeatTime ){
      heartbeat_time = time;
      Serial.print('\2');         //Start Packet
      Serial.print('\x02');       //Two Bytes
      Serial.print('\x01');       //Packet Id:1
      if( emergency_shutoff ) {
        Serial.print('\x00');     //Emergency Shutoff
      } else if( motor_on ) {
        Serial.print('\x01');     //Motor on
      } else { 
        Serial.print('\x02');     //Motor off
      }
    }

    //We need to ack that we received motor_on and motor_off commands.
    if( !motor_on && !m_off_ack ) {
      Serial.print('\2');         //Start Packet
      Serial.print('\x02');       //Two Bytes
      Serial.print('\x02');       //Packet Id:2
      Serial.print('\x00');       //Acknowledge we received a motor off packet.
      m_off_ack = true;           //Don't need to repeatedly send it. If packet is lost, heartbeat will compensate.
    }
    if( motor_on  && !m_on_ack ) {
      Serial.print('\2');         //Start Packet
      Serial.print('\x02');       //Two Bytes
      Serial.print('\x02');       //Packet Id:2
      Serial.print('\x01');       //Acknowledge we received a motor on packet.
      m_on_ack = true;            //Don't need to repeatedly send it. If packet is lost, heartbeat will compensate.
    }
  }
  
//  delay(10);/
}

