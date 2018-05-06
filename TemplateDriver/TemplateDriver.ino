
#define OutputSerial true
#define MaxPacketSize 40

char inByte = 0;   // for incoming serial data

union DConv{
   double x;
   unsigned char c[sizeof (double)]; 
};
union DConv r_dconv;
union DConv w_dconv;
        


union IConv{
   int x;
   unsigned char c[sizeof (int)]; 
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

void processPacket(){
  //Use serialBuffer
  //TODO for each;
}

void setup() {
  Serial.begin( 9600 );
  Serial.flush();
  delay( 1000 );
}


void loop() {
  //Read Serial for any updates from Master
  while (false && Serial.available() > 0) {
    inByte = Serial.read();
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

    if ( packetSize != -1 && bufferPos == packetSize + 1 ){
      processPacket();
      receivedPackets++;
      resetSerial();
    }
  }



  //Write output back to master
  if(OutputSerial){
    Serial.print('\2');
    Serial.print('\x09');
    Serial.print('\x01');
  }
  
  delay(10);
}

