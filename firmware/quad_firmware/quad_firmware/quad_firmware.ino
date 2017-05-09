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
struct send_values {
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
 
int motor_a, motor_b, motor_c, motor_d;
float roll_PID, pitch_PID, yaw_PID;
float roll_error, pitch_error, yaw_error;
sensors_vec_t orientation;

Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());


void setup() {
  pinMode(8, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  Serial.begin(115200);
  rfBegin(13);
  Serial.println("Found LSM9DS1 9DOF");
  Serial.println("Initilizing...");
  while (!Serial);
  setupSensor();
  Serial.println("Ready.");
}

void loop() {
  read_values();
  set_values();
  lmu_display_values();
  //PID();
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

int roll_calculation() {
  
}

int pitch_calculation() {
  
}

int yaw_calculation() {
  
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


void read_values() {
  rfRead((uint8_t*) (&values), sizeof(struct send_values));
}

void set_values() {
  analogWrite(8, values.throttle);
  analogWrite(3, values.throttle);
  analogWrite(4, values.throttle);
  analogWrite(5, values.throttle);
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

