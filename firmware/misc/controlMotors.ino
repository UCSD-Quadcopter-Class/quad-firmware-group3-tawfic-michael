
/* controlMotors.ino
 * allow us to test the motor drivers
 */

// global variables corresponding to motor pwm signals
int motor1 = PB5; // PB5, top right
int motor2 = PE3; // PE3, bottom right
int motor3 = PE4; // PE4, top left
int motor4 = PE5; // PE5, bottom left

// digital high
//int HIGH = 1;
//int LOW = 0;

void setup() {
  //do nothing
  Serial.begin(9600);
  pinMode(A1, OUTPUT);
  pinMode(PB5, OUTPUT);
}
void loop() {
  // maybe this should be analogWrite?
  //analogWrite(motor1, 238); // channel conducts when input is high. we don't
  //digitalWrite(motor1, HIGH); // channel conducts when input is high. we don't
 //want it to spin at first
   //digitalWrite(motor1, HIGH); // testing suspicions
  //digitalWrite(motor2, LOW) // channel conducts when input is high. we don't
  //want it to spin at first
  //digitalWrite(motor3, LOW) // channel conducts when input is high. we don't
  //want it to spin at first
  //digitalWrite(motor4, LOW) // channel conducts when input is high. we don't
  //want it to spin at first
  //Serial.print("running");
  ///delay(500);
  //digitalWrite(motor1, HIGH);
  //digitalWrite(LED_BUILTIN, HIGH);
  //analogWrite(A1, 1023);
  delay(500);
  //digitalWrite(motor1, LOW); // channel conducts when input is high. we don't
  delay(500);
  Serial.print("test\n");
  Serial.println(LOW);
  Serial.println(HIGH);
}

