// USB Port: A904MI6A
#include <Wire.h>
#include "radio.h"
#include "Adafruit_LSM9DS1.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_Simple_AHRS.h"

//Global Object declarations
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
#define DEBUGGING 1
#define NO_TAKEOFF 100 // the highest value motors can be analogwritten without takeoff

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

volatile float filter_roll, filter_pitch, filter_yaw = 0;

// values for testing comp filter
volatile float filter_test[] = {0.0, 0.0};
volatile float acc[] = {0.0, 0.0};
volatile float gyro[] = {0.0, 0.0};
volatile float filter_out = 0.0;
volatile float high_pass[] = {0.0, 0.0};
volatile float low_pass[] = {0.0, 0.0};

volatile float gyro_offset[] = {0, 0, 0};
sensors_event_t a, m, g, temp1;


// Note to self: we are plsying with pitch when we use the test stand



//functions
void setup() {
  pinMode(MA, OUTPUT);
  pinMode(MB, OUTPUT);
  pinMode(MC, OUTPUT);
  pinMode(MD, OUTPUT);

  Serial.begin(115200); // baud
  rfBegin(13);
  
  while (!Serial);
  setupSensor();

  calibrate_gyro(gyro_offset, g, 1);
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
  if( DEBUGGING ) {
//    Serial.println( "values struct : ");
//    Serial.print("\t"); Serial.print("magic:\t"); Serial.println(values.magic);
//    Serial.print("\t"); Serial.print("yaw:\t"); Serial.println(values.yaw);
//    Serial.print("\t"); Serial.print("throt:\t"); Serial.println(values.throttle);
//    Serial.print("\t"); Serial.print("roll:\t"); Serial.println(values.roll);
//    Serial.print("\t"); Serial.print("pitch:\t"); Serial.println(values.pitch);
//    Serial.print("\t"); Serial.print("pot1:\t"); Serial.println(values.pot1);
//    Serial.print("\t"); Serial.print("pot2:\t"); Serial.println(values.pot2);
//    Serial.print("\t"); Serial.print("bt1:\t"); Serial.println(values.bt1);
//    Serial.print("\t"); Serial.print("bt2:\t"); Serial.println(values.bt2);
  }
}

void PID() {
  roll_PID = roll_calculation();
  pitch_PID = pitch_calculation();
  //yaw_PID = yaw_calculation();
  yaw_PID = 0;
  roll_PID = 0;
  
  motor_a = values.throttle + roll_PID - yaw_PID;
 
  motor_b = values.throttle + pitch_PID + yaw_PID;
 
  motor_c = values.throttle - roll_PID - yaw_PID;
 
  motor_d = values.throttle - pitch_PID + yaw_PID;
  
//  motor_a = values.throttle + pitch_PID;
// 
//  motor_b = values.throttle;
// 
//  motor_c = values.throttle;
// 
//  motor_d = values.throttle + pitch_PID;


  if(DEBUGGING) {
     Serial.print("MA: ");
     Serial.print(motor_a);
     Serial.print(" MB: ");
     Serial.print(motor_b);
     Serial.print(" MC: ");
     Serial.print(motor_c);
     Serial.print(" MD: ");
     Serial.print(motor_d);
     Serial.print("  \n");
  }
  
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
  if(roll_sensor_temp > 36) 
    roll_sensor_temp = 36;
  if(roll_sensor_temp < -35)
    roll_sensor_temp = -35;
    
  //pitch_sensor_temp = map(pitch_sensor_temp, -35, 36, 0, 255);
  sensors_event_t a, m, g, temp1;
  lsm.read();
  lsm.getEvent(&a, &m, &g, &temp1); 
//  float gyro_value = gyro_value + g.gyro.y*delta_t*.001;
//  float filter_new = .02*(gyro_value) + .98*(pitch_sensor_temp);
  filter_roll = .9*(filter_roll + g.gyro.x*delta_t*.001) + .1*(roll_sensor_temp);

  for(int i = 0; i !=  5; i++) {
    roll_error[i+1] = roll_error[i];
  }
  
  roll_error[0] = values.roll - (filter_roll + 131);
  int temp = 0;
  for (int i = 0; i !=  5; i++) {
    temp += roll_error[i];
  }
  temp = temp/2; //normalize
  
  if(temp > 255)
    temp = 225/2;
  if(temp < -255)
    temp = -255/2;

  float delta_e = roll_error[0] - roll_error[1];
  if(DEBUGGING) {
//    Serial.print("Roll Filer: ");
//    Serial.print(filter);
////    Serial.print("Second Term: ");
//    Serial.print(" ");
//    Serial.print(filter_pitch);
//    Serial.print(" Pitch Error Sum: ");
//    Serial.print(temp);
//    Serial.print(" Detla_e (Pitch): ");
//    Serial.print(delta_e);
//    Serial.print(" Current Error: ");
//    Serial.print(pitch_error[0]);
//    Serial.print(" K_p: ");
//    Serial.print(k_p);
//    Serial.print(" K_i: ");
//    Serial.print(k_i);
//    Serial.print(" K_d: ");
//    Serial.print(k_d);
  }
  first = k_p * (roll_error[0]);
  
  second = k_i * (temp);

  third = k_d * (delta_e/delta_t); 
  
  return (first + second + third); 
}

float pitch_calculation() {
  float pitch_sensor_temp;
  float first, second, third, total;

  unsigned long time_variable = millis();
  unsigned long delta_t = time_variable - delta_t_prev_pitch;
  delta_t_prev_pitch = time_variable;
  
  if (ahrs.getOrientation(&orientation)) {
    /* 'orientation' should have valid .roll and .pitch fields */
    //pitch_sensor_temp = orientation.pitch;
    acc[0] = orientation.pitch;
  }
  // We shouldn't use this after the quad is detached from the stand, and using it when
  // the quad is on the stand could give us overconfidence in our results. I think should
  // be removed
//  if(pitch_sensor_temp > 36) 
//    pitch_sensor_temp = 36;
//  if(pitch_sensor_temp < -35)
//    pitch_sensor_temp = -35;

  // read from sensors
  lsm.read();
  lsm.getEvent(NULL, NULL, &g, NULL); 
  
  // calculate new gyro position
  gyro[0] = gyro[0] + (g.gyro.y - gyro_offset[1])*delta_t*0.001;

/* All filters use sampling time of 1/2 sec, based on rough timing of our calls */
//  // filters with x-over at 10 Hz
//  low_pass[0] = 0.7143*acc[0] + 0.7143*acc[1] - 0.4286*low_pass[1];
//  high_pass[0] = 0.2857*gyro[0] - 0.2857*gyro[1] - 0.4286*high_pass[1];

//  // filters with x-over at 2 Hz. worse than at 10 hz. try 20 hz
//  low_pass[0] = 0.3333*acc[0] + 0.3333*acc[1] + 0.3333*low_pass[1];
//  high_pass[0] = 0.6667*gyro[0] - 0.6667*gyro[1] + 0.3333*high_pass[1];

//  // filters with x-over at 20 Hz. try 100 hz?
//  low_pass[0] = 0.8333*acc[0] + 0.8333*acc[1] - 0.6667*low_pass[1];
//  high_pass[0] = 0.1667*gyro[0] - 0.1667*gyro[1] - 0.6667*high_pass[1];

//  // filters with x-over at 100 Hz. 20 hz and this one both work well.
  low_pass[0] = 0.9615*acc[0] + 0.9615*acc[1] - 0.9231*low_pass[1];
  high_pass[0] = 0.03846*gyro[0] - 0.03846*gyro[1] - 0.9231*high_pass[1];

  filter_out = high_pass[0] + low_pass[0];

  // move values back in arrays
  acc[1] = acc[0];
  gyro[1] = gyro[0];
  low_pass[1] = low_pass[0];
  high_pass[1] = high_pass[0];
  for(int i = 0; i !=  5; i++) {
    pitch_error[i+1] = pitch_error[i];
  }

  // calculate errors
  pitch_error[0] = values.pitch - (filter_out + 131);
//  Serial.println(values.pitch);
//  Serial.println( pitch_error[0]);
  int i_error = 0;
  for (int i = 0; i !=  5; i++) {
    i_error += pitch_error[i];
  }
  i_error = i_error/2; //normalize

  // I think we should get rid of this for same reason as above
//  if(temp > 255)
//    temp = 225;
//  if(temp < -255)
//    temp = -255;

  float delta_e = pitch_error[0] - pitch_error[1];
  if(DEBUGGING) {
//    Serial.println(" gyro_reading \t acc_reading ");
//    Serial.print(gyro[0]);   
//    Serial.print("\t\t");      
//    Serial.println(acc[0]);  

//    Serial.println("lp_acc\thp_gyr\tfilter\traw_g\traw_acc");
//    Serial.print(" ");      
//    Serial.print(low_pass[0]);   
//    Serial.print("\t");      
//    Serial.print(high_pass[0]);  
//    Serial.print("\t ");      
//    Serial.print(filter_out);   
//    Serial.print(" \t ");      
//    Serial.print(gyro[1]);   
//    Serial.print(" \t ");      
//    Serial.println(acc[1]);
  }
  first = k_p * (pitch_error[0]);
  
  second = k_i * (i_error);

  third = k_d * (delta_e/delta_t); 

//  first = 2 * (pitch_error[0]);
//  
//  second = 0 * (i_error);
//
//  third = 0 * (delta_e/delta_t); 

//  Serial.print(" pitch_error: ");
//  Serial.print(first);
//  Serial.print("  ");
//  Serial.print(second);
//  Serial.print("  ");
//  Serial.print(third);
//  Serial.print("  ");
//  Serial.println(first+second+third);

//  Serial.println( pitch_error[0]);

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

void calibrate_gyro( float * gyro_offset, sensors_event_t g, float percent) {
  Serial.print("Calibrating gyro...");

  if( ( percent > 1 )||( percent < 0 ) ) {
    Serial.println(" Failure in calibration, gryoscope remains uncalibrated.");
    Serial.println(" Percent power not in bounds. Percent must be between 0 and 1, inclusive");
    return;
  }

  // Send power to motors so we calibrate under correct conditions.
  analogWrite(MA, NO_TAKEOFF*percent);
  analogWrite(MB, NO_TAKEOFF*percent);
  analogWrite(MC, NO_TAKEOFF*percent);
  analogWrite(MD, NO_TAKEOFF*percent);
  
//  Serial.println();

  delay(100); // after declaring struct, take some time before using, or you get trash values
  // calculate average offset
  for( int i = 0; i < 100; i++ ) {
    lsm.read();
    lsm.getEvent(NULL, NULL, &g, NULL); 
//    Serial.print(" Reading: ");
//    Serial.println(g.gyro.y);
    gyro_offset[0] += g.gyro.x/100;
    gyro_offset[1] += g.gyro.y/100;
    gyro_offset[2] += g.gyro.z/100;
//    Serial.print(" readings: ");
//    Serial.print( g.gyro.x );
//    Serial.print("  ");
//    Serial.print( g.gyro.y );
//    Serial.print("  ");
//    Serial.println( g.gyro.z );
    delay(50);
  }
  
  analogWrite(MA, 0);
  analogWrite(MB, 0);
  analogWrite(MC, 0);
  analogWrite(MD, 0);
  
  Serial.print("  Finished. \n  Average Offsets (reading - true value) for x, y, z: ");
  Serial.print(gyro_offset[0]);
  Serial.print("  ");
  Serial.print(gyro_offset[1]);
  Serial.print("  ");
  Serial.println(gyro_offset[2]);
  return;
}

