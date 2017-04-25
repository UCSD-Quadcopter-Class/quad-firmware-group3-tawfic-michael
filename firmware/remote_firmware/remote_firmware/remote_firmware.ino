#include "radio.h"
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
  Serial1.begin(115200);
  rfBegin(13);
  Serial.println("Initilizing...");
  while(!Serial);
  Serial.println("Ready.");
  lcd.clear();
  lcd.home();
}

int pin[4];
int temp;

void loop() {
  lcd.setCursor(0,0);
  lcd.print("T:");
  lcd.setCursor(0,2);
  noInterrupts();
  temp = analogRead(PIN_THROTTLE);
  if (temp <= 215) { //bottom 
    temp = 215;
  }
  if (temp >= 1023) { //top 
    temp = 1023;
  }
  pin[0] = map(temp, 215, 1023, 0, 255);
  lcd.print(pin[0]);
  rfWrite(lowByte(pin[0]));
  interrupts();


//  lcd.setCursor(0,10);
//  lcd.print("R:");
//  lcd.setCursor(0,12);
//  temp = analogRead(PIN_ROLL);
//  if (temp <= 295) { //left
//    temp = 295;
//  }
//  if (temp >= 1023) { //right
//    temp = 1023;
//  }
//  pin[1] = map(temp, 1023, 295, 0, 255);
//  lcd.print(pin[1]);
//  rfWrite(lowByte(pin[1]));
//
//  
//  lcd.setCursor(1,0);
//  lcd.print("Y:");
//  lcd.setCursor(1,2);
//  temp = analogRead(PIN_YAW);
//  if (temp <= 234) { //right 
//    temp = 234;
//  }
//  if (temp >= 1023) { //left
//    temp = 1023;
//  }
//  pin[2] = map(temp, 234, 1023, 0, 255);
//  lcd.print(pin[2]);
//  rfWrite(lowByte(pin[2]));
//
//
//
//  
//  lcd.setCursor(1,10);
//  lcd.print("P:");
//  lcd.setCursor(1,12);
//  temp = analogRead(PIN_PITCH);
//  if (temp <= 1023) { //down
//    temp = 1023;
//  }
//  if (temp >= 270) { //up
//    temp = 270;
//  }
//  pin[3] = map(temp, 1023, 270, 0, 255);
//  lcd.print(pin[3]);
//  rfWrite(lowByte(pin[3]));
 
  delay(500);
  lcd.clear();
  
  
}
