#include <SPI.h>
#include <RH_RF95.h>

#define Debug false

// for feather m0 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#if defined(ESP8266)
  /* for ESP w/featherwing */ 
  #define RFM95_CS  2    // "E"
  #define RFM95_RST 16   // "D"
  #define RFM95_INT 15   // "B"

#elif defined(ESP32)  
  /* ESP32 feather w/wing */
  #define RFM95_RST     27   // "A"
  #define RFM95_CS      33   // "B"
  #define RFM95_INT     12   //  next to A

#elif defined(NRF52)  
  /* nRF52832 feather w/wing */
  #define RFM95_RST     7   // "A"
  #define RFM95_CS      11   // "B"
  #define RFM95_INT     31   // "C"
  
#elif defined(TEENSYDUINO)
  /* Teensy 3.x w/wing */
  #define RFM95_RST     9   // "A"
  #define RFM95_CS      10   // "B"
  #define RFM95_INT     4    // "C"
#endif


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

union IConv{
   int x;
   uint8_t c[sizeof (int)]; 
};
union IConv r_iconv;
union IConv w_iconv;

void DebugPrint(String s){
  if( Debug ) {
    Serial.println(s);
  }
}

void serialPrintInt(int val){
  w_iconv.x = val;
  for (unsigned i = 0; i < sizeof (int); i++){
    Serial.write(w_iconv.c[i]);
  }
}

void setup()
{
  pinMode(LED, OUTPUT);

  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  delay(100);

  DebugPrint("Feather LoRa RX Test!");

  if (!rf95.init()) {
    DebugPrint("LoRa radio init failed");
    while (1);
  }
  DebugPrint("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    DebugPrint("setFrequency failed");
    while (1);
  }
  DebugPrint("Set Freq to: "); DebugPrint(String(RF95_FREQ));

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

void loop()
{
  
  delay(5);
  if (rf95.available())
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {    
      Serial.print('\2');
      Serial.print('\x03');
      Serial.print(buf[0] == 0 ? '\0' : '\1');
      Serial.print(buf[1] == 0 ? '\0' : '\1');
      Serial.print(char(buf[2] * 100 + buf[3] * 10  + buf[4]));
    }
    else
    {
      DebugPrint("Receive failed");
    }
  }
}


