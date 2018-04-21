#define DEBUG // define as "DEBUG" to go into debugging mode with Serial Monitor

// Pin assignments
const int POT = A2;     // Linear potentiometer
const int CONTROL = A3; // Logic for gate driver
String readString;

void setup()
{
#ifdef DEBUG
  Serial.begin(9600); // Start Serial
#endif                // DEBUG

  // Set pin I/O
  pinMode(CONTROL, OUTPUT);
  pinMode(POT, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  //  solenoidControl(294, 100);
  potRead();
  handleSerial();
}


/**
 * Reads serial input. Sets solenoidControl function to jump at the input frequency. If the frequency is negative or 0, stops jumping.
 */
void handleSerial()
{
  //reset input if new input
  if(Serial.available()){
    readString = "";
  }
  
  while (Serial.available()) {
    delay(3);  //delay to allow buffer to fill 
    if (Serial.available() >0) {
      char c = Serial.read();  //gets one byte from serial buffer
      readString += c; //makes the string readString
    } 
  }
  int input = readString.toInt();

//  Serial.println(input);
  if(input>0){
      solenoidControl(input, 100);

  } 
  else{
//      Serial.print("Not Jumping. "+input);
  }
}

/**
 * Controls solenoid's jumping time and how much voltage control from Arduino
 * Input:
 * onecycle - period in ms
 * v_level  - how much voltage for Arduino to put in gate from 0 to 100 (should be 100 usually)
 */
void solenoidControl(int onecycle, int v_level)
{
  int scaled_volts = 255 * (v_level / 100);
  int half_cycle = onecycle / 2;

#ifdef DEBUG
  Serial.print("analogWrite input = [ ");
  Serial.print(scaled_volts);
  Serial.println(" ]");
  Serial.print("half_cycle ( delay input ) = [ ");
  Serial.print(half_cycle);
  Serial.println(" ]");
  Serial.println("HIGH");
#endif // DEBUG

  // Actuate solenoid
  digitalWrite(LED_BUILTIN, HIGH);
  analogWrite(CONTROL, scaled_volts);
  delay(half_cycle);

#ifdef DEBUG
  Serial.println("LOW");
#endif

  // De-actuate solenoid (solenoid lets go of what it's holding onto)
  digitalWrite(LED_BUILTIN, LOW);
  analogWrite(CONTROL, 0);
  delay(half_cycle);
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
void solenoidSweep(int start_val, int end_val, int increment, int jumps, int v_level)
{
#ifdef DEBUG
//  Serial.println("SOLENOID SWEEP START");
#endif // DEBUG

  for (int i = start_val / 2; i < end_val / 2; i = i + increment)
  {
    for (int j = 0; j < jumps; j++)
    {
#ifdef DEBUG
//      Serial.print("Jump # = [ ");
//      Serial.print(j);
//      Serial.println(" ]");
#endif //DEBUG
      solenoidControl(i, v_level);
    }
  }
}

/**
 * Returns readings from linear potentiometer
 */
int potRead(void)
{
  int value = analogRead(POT);

#ifdef DEBUG
//  Serial.print("Potentiometer reading = [ ");
//  Serial.print(value);
//  Serial.println(" ]");
#endif // DEBUG

  return value;
}
