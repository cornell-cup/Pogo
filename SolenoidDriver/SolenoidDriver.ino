#define NOT_DEBUG       // define as "DEBUG" to go into debugging mode with Serial Monitor

// Pin assignments
const int POT = A2;     // Linear potentiometer
const int CONTROL = A3; // Logic for gate driver

void setup() {
  #ifdef DEBUG
  Serial.begin( 9600 ); // Start Serial
  #endif // DEBUG

  // Set pin I/O
  pinMode( CONTROL, OUTPUT );
  pinMode( POT, INPUT );
  pinMode( LED_BUILTIN, OUTPUT );
}

void loop() {
  solenoidControl( 294, 100 );
  potRead();
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
  Serial.println( "analogWrite input = [ ", scaled_volts, " ]" ); // TODO check if this works
  Serial.println( "half_cycle ( delay input ) = [ ", half_cycle, " ]");
  Serial.println( "HIGH" );
  #endif // DEBUG

  // Actuate solenoid
  digitalWrite( LED_BUILTIN, HIGH );
  analogWrite( CONTROL, v_level );
  delay( half_cycle );

  #ifdef DEBUG
  Serial.println( "LOW" );
  #endif

  // De-actuate solenoid (solenoid lets go of what it's holding onto)
  digitalWrite( LED_BUILTIN, LOW );
  analogWrite( CONTROL, v_level );
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
      Serial.println( "Jump # = [ ", j, " ]" );
      #endif //DEBUG
      solenoidControl( i, v_level );
    }
  }
}

/**
 * Returns readings from linear potentiometer
 */
int potRead( void ){
  return analogRead( POT );

  #ifdef DEBUG
  Serial.println( "Potentiometer reading = [ ", analogRead( POT ), " ]" );
  #endif // DEBUG
}
