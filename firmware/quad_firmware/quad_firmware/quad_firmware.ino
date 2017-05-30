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
volatile float roll_error[5], pitch_error[5], yaw_error[5] = {0,0,0,0,0};
volatile sensors_vec_t orientation;
//volatile float roll_sensor, pitch_sensor, yaw_sensor;

volatile Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());
volatile float k_p;
volatile float k_d;
volatile float k_i;
volatile unsigned long t_prev_roll = millis();
volatile unsigned long t_prev_pitch = t_prev_roll;
volatile unsigned long t_prev_yaw = t_prev_roll;
volatile float previous_error_roll, previous_error_pitch, previous_error_yaw;

volatile float filter_roll, filter_pitch, filter_yaw = 0;

// values for comp filters
volatile float roll_acc[2], pitch_acc[2] = {0.0, 0.0};
volatile float roll_gyro[2], pitch_gyro[2] = {0.0, 0.0};
volatile float roll_high_pass[2], pitch_high_pass[2] = {0.0, 0.0};
volatile float roll_low_pass[2], pitch_low_pass[2] = {0.0, 0.0};
volatile float pitch_filter_out, roll_filter_out, yaw_filter_out = 0.0;

volatile float gyro_offset[] = {0, 0, 0};
sensors_event_t a, m, g, temp1;
unsigned long time_variable;
unsigned long delta_t;
int roll_i_error = 0;
int pitch_i_error = 0;
int yaw_i_error = 0;
float first, second, third;
float delta_e;


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

  calibrate_gyro(gyro_offset, g, 1, 50, 50);
}

void loop() {
  read_values();
//  delta_t = millis() - t_prev_pitch;
//  t_prev_pitch = t_prev_pitch + delta_t;
//  Serial.println(delta_t);
//  PID();
  if (values.magic == 73) {
    PID();
    //graphing();
    set_values();
    delay(50);
//    Serial.println(" ");
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
//  roll_PID = roll_calculation();
  pitch_PID = pitch_calculation();
  //yaw_PID = yaw_calculation();
  yaw_PID = 0;
  roll_PID = 0;
//  pitch_PID = 0;

  
  
  motor_a = values.throttle - pitch_PID/3;// - yaw_PID;
 
  motor_b = values.throttle + pitch_PID/3; //+ roll_PID + yaw_PID;
 
  motor_c = values.throttle + pitch_PID/3;// - yaw_PID;
 
  motor_d = values.throttle - pitch_PID/3;//roll_PID + yaw_PID;
  
//  motor_a = values.throttle + pitch_PID/2;
// 
//  motor_b = values.throttle;
// 
//  motor_c = values.throttle;
// 
//  motor_d = values.throttle + pitch_PID;

  
  if(values.throttle == 0) {
    motor_a = 0;
    motor_b = 0;
    motor_c = 0;
    motor_d = 0;
  }

  if( exceeds( motor_a, motor_b, motor_c, motor_d, 255) ) {
    int highVal = absMax( motor_a, motor_b, motor_c, motor_b);
    motor_a = (int) ((float) motor_a/highVal * 255);
    motor_b = (int) ((float) motor_b/highVal * 255);
    motor_c = (int) ((float) motor_c/highVal * 255);
    motor_d = (int) ((float) motor_d/highVal * 255);
  }
//  if( motor_a > 255) {
//    motor_a = 255;
//  }
//  if( motor_b > 255) {
//    motor_b = 255;
//  }
//  if( motor_c > 255) {
//    motor_c = 255;
//  }
//  if( motor_d > 255) {
//    motor_d = 255;
//  }

  

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
  
}

float roll_calculation() {
//  float roll_sensor_temp;
//  float first, second, third;

  delta_t = millis() - t_prev_roll;
  t_prev_roll = t_prev_roll + delta_t;
  
  if (ahrs.getOrientation(&orientation)) {
    /* 'orientation' should have valid .roll and .pitch fields */
    roll_acc[0] = orientation.roll;
  }

  // read from sensors
  lsm.read();
  lsm.getEvent(NULL, NULL, &g, NULL); 
  
  // calculate new gyro position
  roll_gyro[0] = roll_gyro[0] + (g.gyro.x - gyro_offset[0])*delta_t*0.001;

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
  roll_low_pass[0] = 0.9615*roll_acc[0] + 0.9615*roll_acc[1] - 0.9231*roll_low_pass[1];
  roll_high_pass[0] = 0.03846*roll_gyro[0] - 0.03846*roll_gyro[1] - 0.9231*roll_high_pass[1];

  roll_filter_out = roll_high_pass[0] + roll_low_pass[0];
  

  // move values back in arrays
  roll_acc[1] = roll_acc[0];
  roll_gyro[1] = roll_gyro[0];
  roll_low_pass[1] = roll_low_pass[0];
  roll_high_pass[1] = roll_high_pass[0];
  for(int i = 0; i !=  5; i++) {
    roll_error[i+1] = roll_error[i];
  }

  // calculate errors
  roll_error[0] = values.roll - (roll_filter_out + 131);
//  Serial.println(values.roll);
//  Serial.println( roll_error[0]);
  roll_i_error = (roll_i_error/1.5) + roll_error[0];

  delta_e = roll_error[0] - roll_error[1];
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
//    Serial.print(roll_filter_out);   
//    Serial.print(" \t ");      
//    Serial.print(gyro[1]);   
//    Serial.print(" \t ");      
//    Serial.println(acc[1]);
  }
//  first = k_p * (roll_error[0]);
//  
//  second = k_i * (i_error);
//
//  third = k_d * (delta_e/delta_t); 

  first = 2 * (roll_error[0]);
  
  second = 0 * (roll_i_error);

  third = 0 * (delta_e/delta_t); 

//  Serial.print(" roll_error: ");
//  Serial.print(first);
//  Serial.print("  ");
//  Serial.print(second);
//  Serial.print("  ");
//  Serial.print(third);
//  Serial.print("  ");
//  Serial.println(first+second+third);

//  Serial.println( roll_error[0]);

  return (first + second + third); 
}

float pitch_calculation() {
//  float pitch_sensor_temp;
//  float first, second, third;

  delta_t = millis() - t_prev_pitch;
  t_prev_pitch = t_prev_pitch + delta_t;

  if (ahrs.getOrientation(&orientation)) {
    /* 'orientation' should have valid .pitch and .pitch fields */
    pitch_acc[0] = orientation.pitch;
  }

  // read from sensors
  lsm.read();
  lsm.getEvent(NULL, NULL, &g, NULL); 
  
  // calculate new gyro position
  pitch_gyro[0] = pitch_gyro[0] + (g.gyro.x - gyro_offset[0])*delta_t*0.001;


// How to make a comp filter:
/*
  1. choose x-over freq.
  
  2. make high pas and low pass filter with this x-over freq. in octave, this can be done by entering:
  >> pkg load control
  >> lp = tf([freq],[1 freq])
  >> hp = tf([1 0],[1 freq])
    (with freq replaced with your x-over freq)

  3. Convert from s-domain to z-domain
  >> c2d( lp, sampling_time, 'tustin')
  >> c2d( hp, sampling_time, 'tustin')
    (with sampling_time replaced with the amount of time passing between calls to the filter (if variable, 90% accuracy of sampling_time is probably good enough) )

  4. The output should be something like this:

        A z + B
   y1:  --------
         z + C  
         (NOTE: A, B, and C might be negative!!! MAKE SURE YOU GET THE SIGNS CORRECT!!)
         (NOTE: don't round constants A, B, and C, use them to as many digits as you are given)

   5. The corresponding filter is:
   filter[0] = A*input[0] + B*input[1] - C*filter[1]
   ( with filter[0] being filter output, filter[1] being filter output one step ago, input[0] being sensor output, and input[1] being sensor output one step ago)
       
  */
/* All these filters use sampling time of 1/2 sec, based on rough timing of our calls */
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
//  pitch_low_pass[0] = 0.9615*pitch_acc[0] + 0.9615*pitch_acc[1] - 0.9231*pitch_low_pass[1];
//  pitch_high_pass[0] = 0.03846*pitch_gyro[0] - 0.03846*pitch_gyro[1] - 0.9231*pitch_high_pass[1];


/* All these filters use sampling time of 50 ms, or 0.05 sec, based on rough timing of our calls */
//  // filters with x-over at 100 Hz.
//  pitch_low_pass[0] = 0.7143*pitch_acc[0] + 0.7143*pitch_acc[1] - 0.4286*pitch_low_pass[1];
//  pitch_high_pass[0] = 0.2857*pitch_gyro[0] - 0.2857*pitch_gyro[1] - 0.4286*pitch_high_pass[1];

//  // filters with x-over at 20 Hz.
//  pitch_low_pass[0] = 0.3333*pitch_acc[0] + 0.3333*pitch_acc[1] + 0.3333*pitch_low_pass[1];
//  pitch_high_pass[0] = 0.6667*pitch_gyro[0] - 0.6667*pitch_gyro[1] + 0.3333*pitch_high_pass[1];

  // filters with x-over at 10 Hz.
  pitch_low_pass[0] = 0.2*pitch_acc[0] + 0.2*pitch_acc[1] + 0.6*pitch_low_pass[1];
  pitch_high_pass[0] = 0.8*pitch_gyro[0] - 0.8*pitch_gyro[1] + 0.6*pitch_high_pass[1];

//  // filters with x-over at 2 Hz.
//  pitch_low_pass[0] = 0.04762*pitch_acc[0] + 0.04762*pitch_acc[1] + 0.9048*pitch_low_pass[1];
//  pitch_high_pass[0] = 0.9524*pitch_gyro[0] - 0.9524*pitch_gyro[1] + 0.9048*pitch_high_pass[1];

//  // filters with x-over at 0.5 Hz.
//  pitch_low_pass[0] = 0.01235*pitch_acc[0] + 0.01235*pitch_acc[1] + 0.9753*pitch_low_pass[1];
//  pitch_high_pass[0] = 0.9877*pitch_gyro[0] - 0.9877*pitch_gyro[1] + 0.9753*pitch_high_pass[1];

  pitch_filter_out = pitch_high_pass[0] + pitch_low_pass[0];
  float pitch = pitch_filter_out;

  // move values back in arrays
  pitch_acc[1] = pitch_acc[0];
  pitch_gyro[1] = pitch_gyro[0];
  pitch_low_pass[1] = pitch_low_pass[0];
  pitch_high_pass[1] = pitch_high_pass[0];
  for(int i = 0; i !=  5; i++) {
    pitch_error[i+1] = pitch_error[i];
  }

  // calculate errors
//  pitch_error[0] = values.pitch - (pitch_filter_out + 131);
  pitch_error[0] = values.pitch - (pitch + 131);
//  Serial.println(values.pitch);
//  Serial.println( pitch_error[0]);
  pitch_i_error = (pitch_i_error/1.5) + pitch_error[0];

  delta_e = pitch_error[0] - pitch_error[1];
  if(DEBUGGING) {
//    Serial.println(" gyro_reading \t acc_reading ");
//    Serial.print(gyro[0]);   
//    Serial.print("\t\t");      
//    Serial.println(acc[0]);  


//    Serial.println("lp_acc\thp_gyr\tfilter\traw_g\traw_acc");
//    Serial.print(" ");      
//    Serial.print(pitch_low_pass[0]);   
//    Serial.print("\t");      
//    Serial.print(pitch_high_pass[0]);  
//    Serial.print("\t ");      
//    Serial.print( pitch );
//    Serial.print(" \t ");      
//    Serial.print(pitch_gyro[1]);   
//    Serial.print(" \t ");      
//    Serial.print(pitch_acc[1]);
//    Serial.println();
  }
  first = 0.75 * k_p * (pitch_error[0]);
  
  second = 0.2 * k_i * (pitch_i_error);

  third = 0.2 * k_d * (delta_e/delta_t); 

//  first = 0.5 * (pitch_error[0]);
//  
//  second = 0.1 * (pitch_i_error);
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


// TODO: put yaw_calculation here


void set_values() {
  analogWrite(MA, motor_a);
  analogWrite(MB, motor_b);
  analogWrite(MC, motor_c);
  analogWrite(MD, motor_d);
  
  if (values.bt2 == 1) {
    k_i = 0.2*values.pot1;
    k_d = 0.2*values.pot2;
  }
  else {
    k_i = 0.2*values.pot1;
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

void calibrate_gyro( float * gyro_offset, sensors_event_t g, float percent, int sample_size, int delay_time) {
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
  for( int i = 0; i < sample_size; i++ ) {
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
    delay(delay_time);
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

int exceeds( int a, int b, int c, int d, int limit) {
  if( (a > limit)||(b > limit)||(c > limit)||(d > limit) ) {
    return 1;
  }
  return 0;
}

int absMax( int a, int b, int c, int d) {
  a = max(a, b);
  c = max(c, d);
  return max(a,c);
}
