
/* controlMotors.ino
 * allow us to test the motor drivers
 */

// global variables corresponding to motor pwm signals
//int motor1 = PB5; // PB5, top right
//int motor2 = PE3; // PE3, bottom right
//int motor3 = PE4; // PE4, top left
//int motor4 = PE5; // PE5, bottom left

void setup() {
  //do nothing
  Serial.begin(9600);
  //pinMode(A1, OUTPUT); // A2 of maybe A1?
  pinMode(8, OUTPUT);
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
  analogWrite( 8, 0); //B5 is actually pin 8, not pin 41. wtf, why? :(
  

  Serial.print("test\n");
  Serial.println("sending 0 to pin 8");
}

