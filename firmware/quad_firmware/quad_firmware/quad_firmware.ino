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

//Global Variables declarations
volatile struct send_values {
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

#define MA 8
#define MB 5
#define MC 4
#define MD 3

// Create a simple AHRS from an explicit accelerometer and magnetometer sensor.
//Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag(), &lsm.getGyro());

/*  Quadcopter X Oritenation
 *           c   d
 *            \-/
 *            /-\
 *           b   a
 */   
 
volatile int motor_a, motor_b, motor_c, motor_d;
volatile float roll_PID, pitch_PID, yaw_PID;
volatile float roll_error[15], pitch_error[15], yaw_error[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile sensors_vec_t orientation;
volatile float roll_sensor, pitch_sensor, yaw_sensor;

volatile Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());
volatile int counter = 0;
volatile float k_p, k_d;
volatile float k_i = .5;
volatile unsigned long delta_t_prev_roll, delta_t_prev_pitch, delta_t_prev_yaw = 0;



//functions
void setup() {
  pinMode(MA, OUTPUT);
  pinMode(MB, OUTPUT);
  pinMode(MC, OUTPUT);
  pinMode(MD, OUTPUT);

  Serial.begin(115200);
  rfBegin(13);
  Serial.println("Found LSM9DS1 9DOF");
  Serial.println("Initilizing...");
  while (!Serial);
  setupSensor();
  Serial.println("Ready.");
  /*
   * if(magic !+ #)
   *    while(1);
   */
}

void loop() {
  read_values();
  read_rpy_sensors();
  PID();
  set_values();
  graphing();
  //lmu_display_values(); //used for debugging purposes
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
  rfRead((uint8_t*) (&values), sizeof(struct send_values));
}

void read_rpy_sensors() {
  if (ahrs.getOrientation(&orientation)) {
    /* 'orientation' should have valid .roll and .pitch fields */
    roll_sensor = orientation.roll*.001; //1 deg/s == .001 deg/ms
    pitch_sensor = orientation.pitch*.001;
    yaw_sensor = orientation.heading*.001;
  }
}

void PID() {
  roll_PID = roll_calculation();
  pitch_PID = pitch_calculation();
  yaw_PID = yaw_calculation();

  motor_a = values.throttle + roll_PID - yaw_PID;
  motor_b = values.throttle + pitch_PID + yaw_PID;
  motor_c = values.throttle - roll_PID - yaw_PID;
  motor_d = values.throttle - pitch_PID + yaw_PID;
}

float roll_calculation() {
  float roll_sensor_temp;
  float first, second, third, total;

  unsigned long delta_t = millis() - delta_t_prev_roll;
  delta_t_prev_roll = delta_t;
  
  if (ahrs.getOrientation(&orientation)) {
    /* 'orientation' should have valid .roll and .pitch fields */
    roll_sensor_temp = orientation.roll;
  }

  sensors_event_t a, m, g, temp1;
  lsm.getEvent(&a, &m, &g, &temp1); 
  
  float filter = .98*(roll_sensor_temp + g.gyro.x*delta_t) + .02*(a.acceleration.x);
  float error_temp = (roll_sensor - values.roll)*filter;
  
  for(int i = 0; i != 13; i++) {
    roll_error[i+1] = roll_error[i];
  }
  
  roll_error[0] = roll_sensor - values.roll;
  int temp = 0;
  for (int i = 0; i != 14; i++) {
    temp += roll_error[0];
  }
  temp = temp/15; //normalize

  float delta_e = temp - error_temp;

  if (temp < 1 && temp > -1) 
    temp = 0;

  if(temp > 255/2)
    temp = 225/2;
  if(temp < -255/2)
    temp = -255/2;

  first = k_p * (0 - error_temp);
  
  second = k_i * (0 - temp);

  third = k_d * (0 - delta_e); 
  
  return (first + second + third); 
}


float pitch_calculation() {
  float pitch_sensor_temp;
  float first, second, third, total;

  unsigned long delta_t = millis() - delta_t_prev_pitch;
  delta_t_prev_pitch = delta_t;
  
  if (ahrs.getOrientation(&orientation)) {
    /* 'orientation' should have valid .roll and .pitch fields */
    pitch_sensor_temp = orientation.pitch;
  }

  sensors_event_t a, m, g, temp1;
  lsm.getEvent(&a, &m, &g, &temp1); 
  
  float filter = .98*(pitch_sensor_temp + g.gyro.x*delta_t) + .02*(a.acceleration.x);
  float error_temp = (pitch_sensor - values.pitch)*filter;
  
  for(int i = 0; i != 13; i++) {
    pitch_error[i+1] = pitch_error[i];
  }
  
  pitch_error[0] = pitch_sensor - values.pitch;
  int temp = 0;
  for (int i = 0; i != 14; i++) {
    temp += pitch_error[0];
  }
  temp = temp/15; //normalize

  float delta_e = temp - error_temp;
  
  if (temp < 1 && temp > -1) 
    temp = 0; 

  if(temp > 255/2)
    temp = 225/2;
  if(temp < -255/2)
    temp = -255/2;

  first = k_p * (0 - error_temp);
  
  second = k_i * (0 - temp);

  third = k_d * (0 - delta_e); 
  
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
  lsm.getEvent(&a, &m, &g, &temp1); 
  
  float filter = .98*(yaw_sensor_temp + g.gyro.x*delta_t) + .02*(a.acceleration.x);
  float error_temp = (yaw_sensor - values.yaw)*filter;
  
  for(int i = 0; i != 13; i++) {
    yaw_error[i+1] = yaw_error[i];
  }
  
  yaw_error[0] = yaw_sensor - values.yaw;
  int temp = 0;
  for (int i = 0; i != 14; i++) {
    temp += yaw_error[0];
  }
  temp = temp/15; //normalize

  float delta_e = temp - error_temp;
  
  if (temp < 1 && temp > -1) 
    temp = 0;
  
  if(temp > 255/2)
    temp = 225/2;
  if(temp < -255/2)
    temp = -255/2;

  first = k_p * (0 - error_temp);
  
  second = k_i * (0 - temp);

  third = k_d * (0 - delta_e); 
  
  return (first + second + third); 
}

void set_values() {
  analogWrite(MA, motor_a);
  analogWrite(MB, motor_b);
  analogWrite(MC, motor_c);
  analogWrite(MD, motor_d);
}

void graphing() {
  Serial.print(roll_PID);
  Serial.print(pitch_PID);
  Serial.print(yaw_PID);
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
