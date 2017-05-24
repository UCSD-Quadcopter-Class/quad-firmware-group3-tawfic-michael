// USB Port: A904MI6A
#include <Wire.h>
#include <radio.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Simple_AHRS.h>

//Global Object declarations
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
#define DEBUGGING 1

//Global Variables declarations
volatile struct receive_values {
  int magic;
  int yaw;
  int throttle;
  int roll;
  int pitch;
  int pot1;
  int pot2;
  int bt1;
  int bt2;
} values;

#define MA 3 //motor a
#define MB 5 //motor b
#define MC 4 //motor c
#define MD 8 //motor d

/*  Quadcopter X Oritenation
 *           Front 
 *          MD   MC
 *            \-/
 *            /-\
 *          MA   MB
 */   
 
volatile int motor_a, motor_b, motor_c, motor_d;
volatile float roll_PID, pitch_PID, yaw_PID;
volatile float roll_error[30], pitch_error[30], yaw_error[30] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile sensors_vec_t orientation;
volatile float roll_sensor, pitch_sensor, yaw_sensor;

volatile Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());
volatile int counter = 0;
volatile float k_p;
volatile float k_d;
volatile float k_i;
volatile unsigned long delta_t_prev_roll, delta_t_prev_pitch, delta_t_prev_yaw = 0;
volatile float previous_error_roll, previous_error_pitch, previous_error_yaw;



//functions
void setup() {
  pinMode(MA, OUTPUT);
  pinMode(MB, OUTPUT);
  pinMode(MC, OUTPUT);
  pinMode(MD, OUTPUT);

  Serial.begin(115200);
  rfBegin(13);
  
  while (!Serial);
  setupSensor();
}

void loop() {
  read_values();
  if (values.magic == 73) {
    PID();
    //graphing();
    set_values();
    delay(500);
    Serial.println(" ");
    //lmu_display_values(); //used for debugging purposes
  }
}

void setupSensor()
{
  lsm.begin();
  delay(250);
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

void read_values() {
  rfRead((uint8_t*) (&values), sizeof(struct receive_values));
}

void PID() {
  roll_PID = roll_calculation();
  pitch_PID = pitch_calculation();
  //yaw_PID = yaw_calculation();
  yaw_PID = 0;

  motor_a = values.throttle + roll_PID - yaw_PID;
 
  motor_b = values.throttle + pitch_PID + yaw_PID;
 
  motor_c = values.throttle - roll_PID - yaw_PID;
 
  motor_d = values.throttle - pitch_PID + yaw_PID;
 


//  if(DEBUGGING) {
//     Serial.print("MA: ");
//     Serial.print(motor_a);
//     Serial.print(" MB: ");
//     Serial.print(motor_b);
//     Serial.print(" MC: ");
//     Serial.print(motor_c);
//     Serial.print(" MD: ");
//     Serial.print(motor_d);
//  }
//  
  if(values.throttle == 0) {
    motor_a = 0;
    motor_b = 0;
    motor_c = 0;
    motor_d = 0;
  }
}

float roll_calculation() {
  float roll_sensor_temp;
  float first, second, third, total;

  unsigned long time_varialbe = millis();
  unsigned long delta_t = time_varialbe - delta_t_prev_roll;
  delta_t_prev_roll = time_varialbe;
  
  if (ahrs.getOrientation(&orientation)) {
    /* 'orientation' should have valid .roll and .pitch fields */
    roll_sensor_temp = orientation.roll;
  }
  
  if(roll_sensor_temp < -40)
    roll_sensor_temp = -40;
  if(roll_sensor_temp > 38)
    roll_sensor_temp = 38;

  roll_sensor_temp = map(roll_sensor_temp, 38, -40, 0, 255);
  
  sensors_event_t a, m, g, temp1;
  lsm.read();
  lsm.getEvent(&a, &m, &g, &temp1); 

  //float filter = .98*(roll_sensor_temp + g.gyro.x*delta_t) + .02*(a.acceleration.x);

  float filter = .98*(filter + g.gyro.x*delta_t) + .02*(roll_sensor_temp);
  
  for(int i = 0; i !=  28; i++) {
    roll_error[i+1] = roll_error[i];
  }
  
  roll_error[0] = roll_sensor_temp - (values.roll) * filter;
  int temp = 0;
  for (int i = 0; i !=  29; i++) {
    temp += roll_error[i];
  }
  
  temp = temp/30; //normalize
  
  if(temp > 255)
    temp = 225/2;
  if(temp < -255)
    temp = -255/2;

  float delta_e = roll_error[0] - roll_error[1];
//  if(DEBUGGING) {
//////    Serial.print(" Roll Error Sum: ");
//////    Serial.print(temp);
//////    Serial.print(" Detla_e (roll): ");
//////    Serial.print(delta_e);
////      Serial.print(" Filter: ");
////      Serial.println(filter);
////      Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
////      Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
////      Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");
////
////       Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
////       Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
////       Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");
////      
//  }
  
  first = k_p * (123 - roll_error[0]);
  
  second = k_i * (123 - temp);

  third = k_d * (123 - (delta_e/delta_t)); 
  
  return (first + second + third); 
}

float pitch_calculation() {
  float pitch_sensor_temp;
  float first, second, third, total;

  unsigned long time_varialbe = millis();
  unsigned long delta_t = time_varialbe - delta_t_prev_pitch;
  delta_t_prev_pitch = time_varialbe;
  
  if (ahrs.getOrientation(&orientation)) {
    /* 'orientation' should have valid .roll and .pitch fields */
    pitch_sensor_temp = orientation.pitch;
  }
  if(pitch_sensor_temp > 36) 
    pitch_sensor_temp = 36;
  if(pitch_sensor_temp < -35)
    pitch_sensor_temp = -35;
    
  //pitch_sensor_temp = map(pitch_sensor_temp, -35, 36, 0, 255);
  sensors_event_t a, m, g, temp1;
  lsm.read();
  lsm.getEvent(&a, &m, &g, &temp1); 
  
  float filter = .02*(filter + g.gyro.y*delta_t*.001) + .98*(pitch_sensor_temp);

  for(int i = 0; i !=  5; i++) {
    pitch_error[i+1] = pitch_error[i];
  }
  
  pitch_error[0] = (filter + 131) - (values.pitch);
  int temp = 0;
  for (int i = 0; i !=  5; i++) {
    temp += pitch_error[i];
  }
  temp = temp/2; //normalize
  
  if(temp > 255)
    temp = 225/2;
  if(temp < -255)
    temp = -255/2;

  float delta_e = pitch_error[0] - pitch_error[1];
  if(DEBUGGING) {
    Serial.print(" Pitch Error Sum: ");
    Serial.print(temp);
    Serial.print(" Detla_e (Pitch): ");
    Serial.print(delta_e);
    Serial.print(" Current Error: ");
    Serial.print(pitch_error[0]);
    Serial.print(" K_p: ");
    Serial.print(k_p);
    Serial.print(" K_i: ");
    Serial.print(k_i);
    Serial.print(" K_d: ");
    Serial.print(k_d);
  }
  first = k_p * (pitch_error[0]);
  
  second = k_i * (temp);

  third = k_d * (delta_e/delta_t); 
  
  return (first + second + third); 
}

float yaw_calculation() {
  float yaw_sensor_temp;
  float first, second, third, total;

  unsigned long delta_t = millis() - delta_t_prev_yaw;
  delta_t_prev_yaw = delta_t;
  
  if (ahrs.getOrientation(&orientation)) {
    /* 'orientation' should have valid .roll and .pitch fields */
    yaw_sensor_temp = orientation.heading;
  }

  sensors_event_t a, m, g, temp1;
  lsm.read();
  lsm.getEvent(&a, &m, &g, &temp1); 
  
  float filter = .98*(yaw_sensor_temp + g.gyro.x*delta_t) + .02*(a.acceleration.x);
  
  for(int i = 0; i !=  28; i++) {
    yaw_error[i+1] = yaw_error[i];
  }
  
  yaw_error[0] = yaw_sensor_temp - (values.yaw) * filter;
  int temp = 0;
  for (int i = 0; i !=  29; i++) {
    temp += yaw_error[0];
  }
  temp = temp/30; //normalize
  
  if(temp > 255)
    temp = 225/2;
  if(temp < -255)
    temp = -255/2;

  float delta_e = yaw_error[1] - yaw_error[0];

  first = k_p * (yaw_error[0]);
  
  second = k_i * (temp);

  third = k_d * ((delta_e/delta_t)); 
  
  return (first + second + third); 
}

void set_values() {
  analogWrite(MA, motor_a);
  analogWrite(MB, motor_b);
  analogWrite(MC, motor_c);
  analogWrite(MD, motor_d);
  
  if (values.bt2 == 1) {
    k_i = values.pot1;
    k_d = values.pot2;
  }
  else {
    k_i = values.pot1;
    k_p = values.pot2;
  }
}

void graphing() {
  //
}

void lmu_display_values() { 
  lsm.read();

  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp);

  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

  Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");

  Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
  Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
  Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");

  Serial.println();
  delay(200);
}
