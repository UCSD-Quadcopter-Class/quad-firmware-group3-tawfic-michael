//A904MI1U

#include "radio.h"
#include "serLCD.h"
#include "quad_remote.h"

//Global Objects declarations
serLCD lcd;

//Global Variables declarations
struct send_values {
  int magic = 73;
  int yaw;
  int throttle;
  int roll;
  int pitch;
  int pot1;
  int pot2;
  int bt1;
  int bt2;
} values;
int prev_button_one = 0;
int prev_button_two = 0;


void setup() {
  pinMode(PIN_YAW, INPUT); //Yaw
  pinMode(PIN_THROTTLE, INPUT); //Throttle
  pinMode(PIN_ROLL, INPUT); //Roll
  pinMode(PIN_PITCH, INPUT); //Pitch
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_GRN, OUTPUT);
  pinMode(PIN_BTN1, INPUT_PULLUP); //blue
  pinMode(PIN_BTN2, INPUT_PULLUP); //green
  pinMode(PIN_POT1, INPUT); // left
  pinMode(PIN_POT2, INPUT); // right
  Serial.begin(115200);
  rfBegin(13);
  
  while(!Serial);
 
  initiate(); //toggels so copter doesnt start on startup
}


void loop() {
  read_values();
  send_values();
  lcd.clear();
  print_lcd(); //1 displays gimble values, 2 displays pot and led values 
}

void initiate() {
  Serial.print("Entered");
  int toggle = 0;
  int tog = 0;
  int t1 = 0;
  int t2 = 0;
  int pin_check[4];
  if(!tog) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Toggle 1: ");
    lcd.setCursor(0,13);
    lcd.print("-");
    lcd.setCursor(1,0);
    lcd.print("Toggle 2: ");
    lcd.setCursor(1,13);
    lcd.print("-");
    tog = 1;
  }
  while(!toggle) {
    //throttle check
    pin_check[0] = analogRead(PIN_THROTTLE);
    if (pin_check[0] <= 119) { //bottom 
      pin_check[0] = 119;
    }
    if (pin_check[0] >= 816) { //top 
      pin_check[0] = 816;
    }
    pin_check[0] = map(pin_check[0], 119, 816, 0, 255);

    //roll check
    pin_check[1] = analogRead(PIN_ROLL);
    if (pin_check[1] <= 120) { //left
      pin_check[1] = 120;
    }
    if (pin_check[1] >= 816) { //right
      pin_check[1] = 816;
    }
    pin_check[1] = map(pin_check[1], 816, 120, 0, 255);

    //yaw check
    pin_check[2] = analogRead(PIN_YAW);
    if (pin_check[2] <= 120) { //right 
      pin_check[2] = 120;
    }
    if (pin_check[2] >= 816) { //left
      pin_check[2] = 816;
    }
    pin_check[2] = map(pin_check[2], 120, 816, 0, 255);

    //pitch check
    pin_check[3] = analogRead(PIN_PITCH);
    if (pin_check[3] >= 816) { //down
      pin_check[3] = 816;
    }
    if (pin_check[3] <= 112) { //up
      pin_check[3] = 112;
    }
    pin_check[3] = map(pin_check[3], 816, 112, 0, 255);
      
    if(pin_check[0] == 0 && pin_check[1] == 0 && pin_check[2] == 255 && pin_check[3] == 0) {
        t1 = 1;
        Serial.println("Check 1");
        lcd.setCursor(0,12);
        lcd.print("DONE");
    }

     if(pin_check[0] == 0 && pin_check[1] == 255 && pin_check[2] == 0 && pin_check[3] == 0 && t1 == 1) {
        t2 = 1;
        Serial.println("Check 2");
        lcd.setCursor(1,12);
        lcd.print("DONE");
    }
  
    if(t1 == 1 && t2 == 1) {
      delay(350);
      toggle = 1;
      lcd.clear();
      lcd.setCursor(0,5);
      lcd.print("Toggle");
      lcd.setCursor(1,4);
      lcd.print("Complete");
      Serial.println("Toggled");
      delay(2000);
    }
  }
}

void read_values() {
  int temp, temp_check;
  
  //read Throttle
  temp = analogRead(PIN_THROTTLE);
  if (temp <= 119) { //bottom 
    temp = 119;
  }
  if (temp >= 816) { //top 
    temp = 816;
  }
  values.throttle = map(temp, 119, 816, 0, 255);

  //read Roll
  temp = analogRead(PIN_ROLL);
  if (temp <= 120) { //left
    temp = 120;
  }
  if (temp >= 816) { //right
    temp = 816;
  }
  values.roll = map(temp, 816, 120, 0, 255);

  //read YAW
  temp = analogRead(PIN_YAW);
  if (temp <= 120) { //right 
    temp = 120;
  }
  if (temp >= 816) { //left
    temp = 816;
  }
  values.pitch = map(temp, 120, 816, 0, 255);

  //read Pitch
  temp = analogRead(PIN_PITCH);
  if (temp >= 816) { //down
    temp = 816;
  }
  if (temp <= 112) { //up
    temp = 112;
  }
  values.pitch = map(temp, 816, 112, 0, 255);

  //read potentiometer 1 
  temp = analogRead(PIN_POT1);
  if(temp <= 111) {
    temp = 111;
  }
  if(temp >= 816) {
    temp = 816;
  }
  values.pot1 = map(temp, 111, 816, 0, 10);

  //read potentiometer 2
  temp = analogRead(PIN_POT2);
  if(temp <= 111) {
    temp = 111;
  }
  if(temp >= 816) {
    temp = 816;
  }
  values.pot2 = map(temp, 111, 816, 0, 10);

  temp = !digitalRead(PIN_BTN1);
  delay(25); //debouncing 
  temp_check = !digitalRead(PIN_BTN1);
  if (temp == temp_check) {
    if (temp != prev_button_one) {
      prev_button_one = temp;
      if (temp == 1 && values.bt1 == 0) {
        values.bt1 = 1;
      }
      else if (temp == 1 && values.bt1 == 1) {
        values.bt1 = 0;
      }
    }
  }
  
  temp = !digitalRead(PIN_BTN2);
  delay(25); //debouncing 
  temp_check = !digitalRead(PIN_BTN2);
  if (temp == temp_check) {
    if (temp != prev_button_two) {
      prev_button_two = temp;
      if (temp == 1 && values.bt2 == 0) {
        values.bt2 = 1;
      }
      else if (temp == 1 && values.bt2 == 1) {
        values.bt2 = 0;
      }
    }
  }

}

void send_values() {
  rfWrite((uint8_t*) (&values), sizeof(struct send_values));
}

void print_lcd() {
  Serial.print(values.bt1);
  if (!values.bt1) { // gimble values == 0
    lcd.setCursor(0,0);
    lcd.print("T:");
    lcd.setCursor(0,2);
    lcd.print(values.throttle);
    lcd.setCursor(0,11);
    lcd.print("R:");
    lcd.setCursor(0,13);
    lcd.print(values.roll);
    lcd.setCursor(1,0);
    lcd.print("Y:");
    lcd.setCursor(1,2);
    lcd.print(values.yaw);
    lcd.setCursor(1,11);
    lcd.print("P:");
    lcd.setCursor(1,13);
    lcd.print(values.pitch);
  } 
  if (values.bt1) { // pot and button values == 1
    lcd.setCursor(0,0);
    lcd.print("POT 1:");
    lcd.setCursor(0,6);
    lcd.print(values.pot1);
    lcd.setCursor(1,0);
    lcd.print("POT 2:");
    lcd.setCursor(1,6);
    lcd.print(values.pot2);  

    if (values.bt2) {
      lcd.setCursor(0,13);
      lcd.print("k_i");
      lcd.setCursor(1,13);
      lcd.print("k_d");
    }
    else {
      lcd.setCursor(0,13);
      lcd.print("k_i");
      lcd.setCursor(1,13);
      lcd.print("k_p");
    }
  }
  delay(300);
  lcd.clear();
}






