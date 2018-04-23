#include "VescUart.h"
#include "datatypes.h"

struct bldcMeasure measuredValues;
unsigned long time;
unsigned long prev_time;
unsigned long tx_time;
unsigned long count;
bool read_rpm = false;
int incomingByte = 0;

void setup() {
  Serial.begin(9600);
  // VESC UART
  Serial1.begin(9600);
  Serial.println("STARTING");
  Serial.flush();
  /* Initialise the sensor */
  delay(1000);
}

// the loop function runs over and over again until power down or reset
void loop() {

//  Serial.println("in loop");
  prev_time = time;
  time = millis();
  //  Serial.println( time - prev_time );
  //  delay(5);

  if ( !read_rpm) {
    VescUartGetValue(measuredValues);
    tx_time = time;
    read_rpm = true;
  }
  else {
    if ( time - tx_time  > 120 ) {
      if ( VescUartReadValue(measuredValues) ) {
        //        Serial.print("Loop: "); Serial.println(count++);
        //        SerialPrint(measuredValues);
        read_rpm = false;
      }
      else {
//        Serial.println("Failed to get data!");
        read_rpm = false;
      }
    }
  }


  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    // say what you got:
    //    Serial.print("I received: ");
    //    Serial.println(incomingByte, DEC);
    if (incomingByte == '0') {
      SerialPrint(measuredValues);
    }
    else {
      Serial.print("I received: TEST INPUT ");
      Serial.println(incomingByte, DEC);

    }
  }


  delay(10);
}

