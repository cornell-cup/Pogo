#include "VescUart.h"
#include "datatypes.h"

unsigned long time;
unsigned long prev_time;
unsigned long tx_time;
unsigned long count;

void setup() {
  Serial.begin(9600);
  // VESC UART
  Serial1.begin(9600);
  delay(1000);
}

struct bldcMeasure measuredValues;

#define MAX_BUFFER_LEN 4
byte bindex = 0;
char buffer[MAX_BUFFER_LEN];

#define MAX_CURRENT 80.0f
float current = 0.0;
bool on = true;
bool read_rpm = false;

// the loop function runs over and over again until power down or reset
void loop() {

  prev_time = time;
  time = millis();
  Serial.println( prev_time );
  if (time - tx_time  > 145){
    if (on) {
      VescUartSetCurrent(0.0);
      on = true;
    }
    else {
      VescUartSetCurrent(MAX_CURRENT);
      on = false;
    }
  }
}

