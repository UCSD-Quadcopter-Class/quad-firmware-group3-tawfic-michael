#include "radio.h"


void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  rfBegin(13);
  Serial.println("Initilizing...");
  while(!Serial);
  Serial.println("Ready.");
}

int input; 

void loop() {
  if(rfAvailable()){
    input = rfRead();
    analogWrite(8, input);
  }
  Serial.println(input);
}
