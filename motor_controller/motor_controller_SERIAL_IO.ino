#include "VescUart.h"
#include "datatypes.h"
//
//struct bldcMeasure measuredValues;
//unsigned long time;
//unsigned long prev_time;
//unsigned long tx_time;
//unsigned long count;
//bool read_rpm = false;
//int incomingByte = 0;

//void setup() {
//  Serial.begin(9600);
//  // VESC UART
//  Serial1.begin(9600);
//  Serial.println("STARTING");
//  Serial.flush();
//  /* Initialise the sensor */
//  delay(1000);
//}

// the loop function runs over and over again until power down or reset
void loop() {
float float_example = 1.11;
byte bytes[4];

float2Bytes(float_example,&bytes[0]);

////  Serial.println("in loop");
//  prev_time = time;
//  time = millis();
//  //  Serial.println( time - prev_time );
//  //  delay(5);
//
//  if ( !read_rpm) {
//    VescUartGetValue(measuredValues);
//    tx_time = time;
//    read_rpm = true;
//  }
//  else {
//    if ( time - tx_time  > 120 ) {
//      if ( VescUartReadValue(measuredValues) ) {
//        //        Serial.print("Loop: "); Serial.println(count++);
//        //        SerialPrint(measuredValues);
//        read_rpm = false;
//      }
//      else {
////        Serial.println("Failed to get data!");
//        read_rpm = false;
//      }
//    }
//  }
//
//
//  if (Serial.available() > 0) {
//    incomingByte = Serial.read();
//
//    // say what you got:
//    //    Serial.print("I received: ");
//    //    Serial.println(incomingByte, DEC);
//    if (incomingByte == '0') {
//      SerialPrint(measuredValues);
//    }
//    else {
//      Serial.print("I received: TEST INPUT ");
//      Serial.println(incomingByte, DEC);
//
//    }
//  }
//

  delay(10);
}


void float2Bytes(float val,byte* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}
