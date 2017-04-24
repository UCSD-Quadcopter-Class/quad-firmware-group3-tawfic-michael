#include <radio.h>
#include "serLCD.h"
#include "quad_remote.h"

//Global Variables
int readYaw = 0;
int readThrottle = 0;
int readRoll = 0;
int readPitch = 0; 
serLCD lcd;

void setup() {
  pinMode(PIN_YAW, INPUT); //Yaw
  pinMode(PIN_THROTTLE, INPUT); //Throttle
  pinMode(PIN_ROLL, INPUT); //Roll
  pinMode(PIN_PITCH, INPUT); //Pitch
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_GRN, OUTPUT);
  pinMode(PIN_BTN1, INPUT);
  pinMode(PIN_BTN2, INPUT);

  Serial.begin(9600);
//  rf.begin(13);
  while(!Serial);
  Serial.print("Ready.\n");
  lcd.clear();
  lcd.home();
}

char *labels[4] = {"T", "Y", "P", "R"};
char pins[4] = {PIN_THROTTLE, PIN_YAW, PIN_PITCH, PIN_ROLL};

void loop() {
  char buf[2];
  lcd.setCursor(0,0);
  lcd.print("T:");
  lcd.setCursor(0,2);
  lcd.print(analogRead(PIN_THROTTLE));
  lcd.setCursor(0,10);
  lcd.print("R:");
  lcd.setCursor(0,12);
  lcd.print(analogRead(PIN_ROLL));
  lcd.setCursor(1,0);
  lcd.print("Y:");
  lcd.setCursor(1,2);
  lcd.print(analogRead(PIN_YAW));
  lcd.setCursor(1,10);
  lcd.print("P:");
  lcd.setCursor(1,12);
  lcd.print(analogRead(PIN_PITCH));
  delay(1000);
  lcd.clear();
  delay(500);
  
}
