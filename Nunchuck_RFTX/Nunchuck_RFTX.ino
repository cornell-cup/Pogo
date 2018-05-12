#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include "Nunchuk.h"

// for feather m0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
// C button will be for reaction wheel start and z will be for jumper start


/* for shield
  #define RFM95_CS 10
  #define RFM95_RST 9
  #define RFM95_INT 7
*/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
uint8_t XJoystick;
uint8_t tensDigit;
uint8_t hundredsDigit;
uint8_t onesDigit;
uint8_t C;
uint8_t Z;
uint8_t data[5];
uint8_t pot;
uint8_t rwStart;
uint8_t jumpStart;
bool powerOff;

void setup()
{
  pot = 0;
  powerOff = false;
  XJoystick = 0;
  C = 0;
  Z = 0;
  Wire.begin();
  nunchuk_init();
  pinMode(A1, INPUT); //C button
  pinMode(A2, INPUT); //Z button
  pinMode(A3, INPUT); //pot
  Serial.begin(115200);

  rwStart = 0;
  jumpStart = 0;

  delay(100);

  //Serial.println("Feather LoRa TX Test!");

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  //Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  //Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

//int16_t packetnum = 0;  // packet counter, we increment per xmission

/*void receiveEvent(int howMany) {
  count = 0;
  while(1 <= Wire.available()){
      data[count] = Wire.read();
      Serial.print(data[count]);
      count++;
    }


  /*String x = Wire.read();    // receive byte as an integer
  Serial.println(x);         // print the integer*/

//}

void loop()
{

  //determining if the shutdown condition is met
  powerOff = false;
  int c = 0;
  while (analogRead(A2) < 20 && analogRead(A1) < 20) {
    c++;
    if (c > 100000) {
      powerOff = true;
      rwStart = 0;
      jumpStart = 0;
      break;
    }
  }

  //determining whether or not c or z button is pressed
  C = 0;
  Z = 0;
  if (analogRead(A1) < 20) {
    C = 1; //C Pressed
  }
  if (analogRead(A2) < 20) {
    Z = 1; //Z Pressed
  }

  //setting the Start variables
  if (!powerOff) {
    //setting rwStart based on C and jumpStart to 0 if rwStart is set to 0
    if (C != 0) {
      if (rwStart == 0) rwStart = 1;
      else {
        rwStart = 0;
        jumpStart = 0;
      }
    }
    //setting jumpStart if the rwStart condition is met
    if ( rwStart == 1 && Z != 0) {
      if (jumpStart == 0) jumpStart = 1;
      else jumpStart = 0;
    }
  }

  //Setting Joystick data
  onesDigit = 0;
  tensDigit = 0;
  hundredsDigit = 0;
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;

  //Serial.println("XJoystick is"); Serial.println(XJoystick);

  pot = analogRead(A3);

  //Serial.println("Pot is"); Serial.println(pot);

  onesDigit = pot % 10;
  pot = pot - onesDigit;
  tensDigit = pot % 100;
  pot = pot - tensDigit;
  tensDigit = tensDigit / 10;
  hundredsDigit = pot % 1000;
  hundredsDigit = hundredsDigit / 100;





  data[0] =  rwStart;
  data[1] =  jumpStart;
  data[2] = hundredsDigit;
  data[3] = tensDigit;
  data[4] = onesDigit;

  //  Serial.println("Z is"); Serial.println(Z);
  //  Serial.println("C is"); Serial.println(C);
  //  Serial.println("Hundreds digit is "); Serial.println(hundredsDigit);
  //  Serial.println("Tens digit is"); Serial.println(tensDigit);
  //  Serial.println("Ones digit is "); Serial.println(onesDigit);

  //  Serial.println("data is"); Serial.print(data[0]); Serial.print(data[1]); Serial.print(data[2]); Serial.print(data[3]); Serial.print(data[4]);
  //XJoystick = Z + C + XJoystick;
  //delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
  //  Serial.println("Transmitting..."); // Send a message to rf95_server

  //char radiopacket[20] = "Hello World #      ";
  //itoa(packetnum++, radiopacket+13, 10);
  // Serial.print("Sending "); //Serial.println(data);
  //radiopacket[19] = 0;

  //Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)data, sizeof(data));

  //Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  //  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  //  uint8_t len = sizeof(buf);

  //Serial.println("Waiting for reply...");
  //  if (rf95.waitAvailableTimeout(1000))
  //  {
  //    // Should be a reply message for us now
  //    if (rf95.recv(buf, &len))
  //    {
  //      Serial.print("Got reply: ");
  //      Serial.println((char*)buf);
  //      Serial.print("RSSI: ");
  //      Serial.println(rf95.lastRssi(), DEC);
  //    }
  //    else
  //    {
  //      Serial.println("Receive failed");
  //    }
  //  }
  //  else
  //  {
  //    Serial.println("No reply, is there a listener around?");
  //  }

}

