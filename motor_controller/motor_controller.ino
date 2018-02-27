#include "VescUart.h"
#include "datatypes.h"

unsigned long count;

void setup() {
  Serial.begin(9600);
  // VESC UART
  Serial1.begin(9600);
  delay(1000);
  pinMode(3, INPUT_PULLUP);
}

struct bldcMeasure measuredValues;

#define MAX_BUFFER_LEN 4
byte bindex = 0;
char buffer[MAX_BUFFER_LEN];

#define MAX_CURRENT 4.0f
#define CURRENT_ACCEL 0.05f
#define CURRENT_DECEL 0.20f
float current = 0.0;
bool accel = true;

// the loop function runs over and over again until power down or reset
void loop() {
  
  if (VescUartGetValue(measuredValues)) {
    Serial.print("Loop: "); Serial.println(count++);
    SerialPrint(measuredValues);
    delay(10);
  }
  else {
    Serial.println("Failed to get data!");
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


//  while (Serial1.available() > 0) {
//    int c = Serial1.read();
//
//    if (c == '\n') {
//      bindex = 0;
//    }
//    else {
//      buffer[bindex] = (char) c;
//      bindex++;
//    }
//
//    if (bindex == MAX_BUFFER_LEN) {
//      // Process buffer
//      int value = (buffer[1] - '0') * 100 + (buffer[2] - '0') * 10 + (buffer[3] - '0');
//      if (buffer[0] == 'S') {
//        // Set motor current forward
//        float current = value / 100.f;
//        if (current < MAX_CURRENT) {
//          Serial1.println("Set forward current");
//          VescUartSetCurrent(current);
//        }
//      }
//      else if (buffer[0] == 'R') {
//        // Set motor current backward
//        float current = value / 100.f;
//        if (current < MAX_CURRENT) {
//          Serial1.println("Set backward current");
//          VescUartSetCurrent(-current);
//        }
//      }
//      else if (buffer[0] == 'B') {
//        // Set brake current
//        float current = value / 100.f;
//        if (current < MAX_CURRENT) {
//          Serial1.println("Set brake current");
//          VescUartSetCurrentBrake(current);
//        }
//      }
//      bindex = 0;
//    }
//  }
}

