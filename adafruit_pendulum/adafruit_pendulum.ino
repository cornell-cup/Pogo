#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int PWMPin = 6;      // LED connected to digital pin 9
//int EN = 6;
int DIRPin = 7;
int ENPin = 8;
int val = 230;         // variable to store the read value; range is 26-230
int valmin = 100;

unsigned long time;
unsigned long prev_time;

short i = 0;
short j = 0;
float pwrfactor = 10;

float theta = 0;
float theta_Prev = 0;
float theta_Speed = 0;
float theta_Speed_Prev = 0;

float theta_Integral = 0;
float theta_Integral_Max = 2;

float power = 26; 


//vars for PID
float error = 0;

float imax = 300;
float iarr[300];
int iptr = 0;
float integral = 0;

// commands from datasheet
const byte NOP = 0x00; // NO oPeration (NOP)
const byte READ_POS = 0x10; // Read position



void setup()

{
  digitalWrite(DIRPin, LOW);
  pinMode(ENPin, OUTPUT);
  digitalWrite(ENPin, HIGH);
  Serial.begin( 9600 );
  Serial.println( "STARTING" );
  Serial.flush();

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay( 2000 );
  bno.setExtCrystalUse(true);

}


void write_power ( int pwm ) {
  //  if ( pwm < 26 && pwm >= 0){
  //      pwm = 26;
  //  }
  //  else if (pwm > -26 && pwm < 0 ) {
  //      pwm = -26;
  //  }
//  Serial.println(pwm);
  if ( pwm > 0) {
    analogWrite(PWMPin, pwm); 
    digitalWrite(DIRPin, LOW);
 
  } else {
    analogWrite(PWMPin, -pwm); 
    digitalWrite(DIRPin, HIGH);
  }
}



void loop()

{ 

   sensors_event_t event;
   bno.getEvent(&event);
   error = event.orientation.y;
   
   error  = error + 3.85 - (1.0 * theta_Integral);
//   Serial.println(error);
    
   if(error > -80 && error < 80 && abs(error-theta) < 4){

//    Serial.print("Time: ");
    prev_time = time;
    time = millis();
    
    //prints time since program started
    Serial.println(time-prev_time);

    theta_Prev = theta;
    theta = error;

    theta_Speed_Prev = theta_Speed;
    theta_Speed = theta - theta_Prev;
//    Serial.println(error);

    if(time-prev_time < 100){
//      theta_Integral += (theta-.1) * (time-prev_time) / 10000.0;
      theta_Integral = constrain(theta_Integral, -theta_Integral_Max, theta_Integral_Max);
    }
//    Serial.print(theta_Integral);
//    Serial.print(",");
//    Serial.print(error);
//    Serial.print(",");
//    Serial.println(theta_Integral);

    
//    integral += error;
//    iptr += 1;
//    if( iptr >= imax ) {
//      iptr = 0;
//    }
//    integral -= iarr[iptr];
//    iarr[iptr] = error;
//
//    Serial.println(integral/imax);
    
//    float amp = abs(error) * 1.404;
//    integral += amp;
//    iptr += 1;
//    if( iptr >= imax ) {
//      iptr = 0;
//    }
//    integral -= iarr[iptr];
//    iarr[iptr] = amp;
//    Serial.println(amp);

    

    float pid = -(0.45 * error) - (38.2* theta_Speed) + (.005 * (power));
    power = power +  pid;
    power = constrain(power, -230, 230);
    write_power(round(power));

   } 
    
//   delayMicroseconds(20);    // datasheet says 20us b/w reads is good

}

