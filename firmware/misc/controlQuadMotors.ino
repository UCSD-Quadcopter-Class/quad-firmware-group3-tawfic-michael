
/* controlMotors.ino
 * allow us to test the motor drivers
 */

// global variables corresponding to motor pwm signals
int motor1 = 8; // PB5, top right
int motor2 = 9; // PE3, bottom right
int motor3 = 10; // PE4, top left
int motor4 = 11; // PE5, bottom left

void setup() {
  //do nothing
  Serial.begin(9600);
  //pinMode(A1, OUTPUT); // A2 of maybe A1?
  // figure out pin numbers just by searching google for pin numbers
  pinMode(motor1, OUTPUT); // B5
  pinMode(motor2, OUTPUT); // B5
  pinMode(motor3, OUTPUT); // B5
  pinMode(motor4, OUTPUT); // B5
  
  int input1 = 0;
  int input2 = 0;
  int input3 = 0;
  int input4 = 0;
}
void loop() {
  // maybe this should be analogWrite?
  //analogWrite(motor1, 238); // channel conducts when input is high. we don't
  //digitalWrite(motor1, HIGH); // channel conducts when input is high. we don't
  //want it to spin at first
  //delay(500);
  //digitalWrite(motor1, HIGH);
  //digitalWrite(LED_BUILTIN, HIGH);
  //analogWrite(A1, 1023);
  //delay(500);
  //digitalWrite(motor1, LOW); // channel conducts when input is high. we don't
  //delay(500);
  //analogWrite( 1, 10);
  analogWrite( motor1, input1);
  analogWrite( motor2, input2);
  analogWrite( motor3, input3);
  analogWrite( motor4, input4);
  

  if( rfAvailable() ) {
    //input1 = rfRead();
    // figure out how we're transmitting data so we can read it in nicely.
    // honestly a simple struct would be nice for this
  }
}

