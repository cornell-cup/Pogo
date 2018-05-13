#define NOT_DEBUG       // define as "DEBUG" to go into debugging mode with Serial Monitor
#define OutputSerial false
#define MaxPacketSize 40

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
void processPacket(){
  if (serialBuffer[0] == '\0' && packetSize == 1)
  {
    // shutoff();
  }
  else if (serialBuffer[0] == '\1' && packetSize == 1)
  {
  // jumps at max
  solenoidControl( 294, 100 );
  }

  //always
  potRead();
}

void setup() {
  #ifdef DEBUG
  Serial.begin( 9600 ); // Start Serial
  #endif // DEBUG

  // Set pin I/O
  pinMode( CONTROL, OUTPUT );
  pinMode( POT, INPUT );
  pinMode( LED_BUILTIN, OUTPUT );
}

//from templateDriver
// void setup() {
//   Serial.begin( 9600 );
//   Serial.flush();
//   delay( 1000 );
// }

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



  //Write output back to master
  if(OutputSerial){
    //First byte is always '\2'
    Serial.print('\2');
    //Second byte is the size of the rest of the packet. Double * 8 + Float * 4 + Int * 4 + Char * 1 
    Serial.print('\x09');
    Serial.print('\x01');
  }
  
  delay(10);
}





/**
 * Controls solenoid's jumping time and how much voltage control from Arduino
 * Input:
 * onecycle - period in ms
 * v_level  - how much voltage for Arduino to put in gate from 0 to 100 (should be 100 usually)
 */
void solenoidControl( int onecycle, int v_level ) {
  int scaled_volts = 255 * ( v_level / 100 );
  int half_cycle = onecycle / 2;

  #ifdef DEBUG
  Serial.print( "analogWrite input = [ " ); Serial.print( scaled_volts ); Serial.println(" ]" );
  Serial.print( "half_cycle ( delay input ) = [ " ); Serial.print( half_cycle ); Serial.println( " ]" );
  Serial.println( "HIGH" );
  #endif // DEBUG

  // Actuate solenoid
  digitalWrite( LED_BUILTIN, HIGH );
  analogWrite( CONTROL, scaled_volts );
  delay( half_cycle );

  #ifdef DEBUG
  Serial.println( "LOW" );
  #endif

  // De-actuate solenoid (solenoid lets go of what it's holding onto)
  digitalWrite( LED_BUILTIN, LOW );
  analogWrite( CONTROL, 0 );
  delay( half_cycle );
}

/**
 * Sweep values for solenoid to bounce, helpful for finding resonance point
 * Input:
 * start_val - beginning period guess in ms
 * end_val   - end period guess in ms
 * increment - how much to increment period guess in ms
 * jumps     - jumps per cycle
 * v_level   - how much voltage for Arduino to put in gate of NFET from 0 to 100 
 *           - should be 100 usually
 */
void solenoidSweep( int start_val, int end_val, int increment, int jumps, int v_level ){
  #ifdef DEBUG
  Serial.println( "SOLENOID SWEEP START" );
  #endif // DEBUG
  
  for ( int i = start_val / 2; i < end_val / 2; i = i + increment ) {
    for ( int j = 0; j < jumps; j++ ) {
      #ifdef DEBUG
      Serial.print( "Jump # = [ "); Serial.print( j ); Serial.println( " ]" );
      #endif //DEBUG
      solenoidControl( i, v_level );
    }
  }
}

/**
 * Returns readings from linear potentiometer
 */
int potRead( void ){
  int value = analogRead( POT );
  
  #ifdef DEBUG
  Serial.print( "Potentiometer reading = [ " ); Serial.print( value ); Serial.println( " ]" );
  #endif // DEBUG
  
  return value;
}
