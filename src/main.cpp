/************************************************************************
 * Author: Micah Janzen
 * Date: 2/3/2022
 * Program Description: This program utilized the BMI160 6 axis accelerometer and gyroscope sensor to 
 * control 2 leds. LEDs are turned on when acceleration is negative and two pushbuttons act as turn signals which
 * make the LEDs blink 
 * Included library(driver code for BMI160): https://github.com/bastian2001/BMI160-Arduino-1
***********************************************************************/
#include <Arduino.h>

#include <BMI160-Arduino/BMI160.h>
#include <BMI160-Arduino/BMI160Gen.h>
#include <BMI160-Arduino/CurieIMU.h>
#include <arduino-timer.h>

const int g_range=2;

/** Functions **/
//takes raw gyro sensor values and maps to +-250 values
float convertRawGyro(int gRaw);
//takes raw acceleration data and maps m/s^2 values
float convertRawAcc(int aRaw);
//returns the magnitude of the x and y component of acceleration
int get_xy_magn(int x, int y);
//takes in array, then returns the average
int get_ma(int* arr);
//takes in arr, shifts each value "left" discarding [0] index, and putting new val at index 20
void update_arr(int* arr,int new_val);
//
bool brake_timer_handler();
bool calc_vel();
bool turn_timer_handler();



auto brake_timer = timer_create_default();
auto sum_acc_timer=timer_create_default();
auto turn_timer=timer_create_default();


/** Global Vars **/
//
int toggle_rate=500; //start out at 1 second toggle rate 
//used to keep track of time similiar to timer
int long last_millis=0;
int long curr_millis=0;
//speed(only in x direction atm) info. used to sum the accelerometer readings over the period of 1 sec
float velocity,prev_velocity=0;


//4 total signals for LEDs, left dim ligh, left bright light, and the same for the right
//LEFT DIM: D5
int left_dim_pin=5;
bool left_dim_val=false;
//LEFT BRIGHT: D6
int left_bright_pin=6;
bool left_bright_val=false;
//RIGHT DIM: D7
int right_dim_pin=7;
bool right_dim_val=false;
//RIGHT BRIGHT: D8
int right_bright_pin=8;
bool right_bright_val=false;

//2 pushbuttons to trigger turn signal
//LEFT BUTTON: D9
int left_btn_pin=9;
bool left_btn_val=false;
bool turn_left=false;
//RIGHT BUTTON D10:
int right_btn_pin=10;
bool right_btn_val=false;
bool turn_right=false;



//dispalcement aimed to keep track of total ditance traveled
int displacement=0;
//var to try and adjust for error at resting place
const float err_adj=.06;
float rotX=0;
float rot_sum_x=0;

/*
Setup Description: Initialize Serial, set 4 led signals as outuput and take pushbutton as input.
Setup BMI160 sensor, using SPI communication between sensor and arduino, and calibrate sensor.
*/
void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open
  //set btn as input, leds as ouptu


  pinMode(left_dim_pin, OUTPUT);
  pinMode(left_bright_pin, OUTPUT);
  pinMode(right_dim_pin, OUTPUT);
  pinMode(right_bright_pin, OUTPUT);
  pinMode(left_btn_pin, INPUT_PULLUP);
  pinMode(right_btn_pin, INPUT_PULLUP);


  // initialize device
  Serial.println("Initializing IMU device...");
  BMI160.begin(BMI160GenClass::I2C_MODE, 0x68);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

  // Set the accelerometer range to 250 degrees/second
  BMI160.setGyroRange(3);
  BMI160.initialize();
  BMI160.setAccelOffsetEnabled(true);
  BMI160.setGyroOffsetEnabled(true);
  BMI160.autoCalibrateGyroOffset();
  BMI160.autoCalibrateXAccelOffset(0);
  BMI160.autoCalibrateYAccelOffset(0);
  //delay needed for autocalibration of offsets
  delay(250);
  //BMI160.setAccelRate(8);

 // BMI160.setAccelRate(32000);
  BMI160.interruptsEnabled(CURIE_IMU_MOTION);
  BMI160.setFullScaleAccelRange(8);
  BMI160.setAccelRate(6);
  BMI160.setGyroRate(6);
  Serial.println(BMI160.getFullScaleAccelRange());
  Serial.println("Initializing IMU device...done.");
  brake_timer.every(1000,brake_timer_handler);
  sum_acc_timer.every(1000,calc_vel);
  turn_timer.every(250,turn_timer_handler);
}

/*
Loop Description: Reads in button values as well as the acceleration+gyroscope raw values from sensor.
if button was pressed, toggle blinking with respective led(left or right). If there is negative acceleration
Turn on led that isn't blinking and let the led that is blinking continue to blink.
*/
void loop() {
  
  float accX,accY, accZ;
  
  
  brake_timer.tick();
  sum_acc_timer.tick();
  turn_timer.tick();
 

  //read button values( ACTIVE LOW)
  left_btn_val=digitalRead(left_btn_pin);
  right_btn_val=digitalRead(right_btn_pin);

//if left button found true, swap turn signal val for left, essentially toggling the turn signal
  if(left_btn_val==0){
    turn_left=(!turn_left);
    if(turn_right==true){
      digitalWrite(right_dim_pin,LOW);
      turn_right=false;
    }
    digitalWrite(left_dim_pin,LOW);
  }
  if(right_btn_val==0){
    turn_right=(!turn_right);
    if(turn_left==true){
      digitalWrite(left_dim_pin,LOW);
      turn_left=false;
    }
    digitalWrite(right_dim_pin,LOW); 
  }


  //add new val to arr of ma for X Y and Z arrs
  //update_arr(acc_maX, (convertRawAcc(BMI160.getAccelerationX())) );
  rotX=convertRawGyro(BMI160.getRotationX());
  if((rotX>15) || (rotX<-15)){
    rot_sum_x+=rotX;
  }
  //Serial.println(rot_sum_x);
  //get ratio of read value against the max value.
  float rot_ratio=rotX/250;
  //based on this ratio, subtract this amount of gravity(9.81) from the read x value
  float grav_accel_x=rot_ratio*9.81;
  //Serial.println(grav_accel_x);

  accX=convertRawAcc(BMI160.getAccelerationX());
  if(accX>1){
    accX+=.6;
  }
  else if(accX<-1){
    accX+=.7;
  }
  //subtract the ratio of rotation to acceleration from gravity from x acceleration to eliminate it
  accX-=grav_accel_x;
  
  accY=convertRawAcc(BMI160.getAccelerationY());
  accZ=convertRawAcc(BMI160.getAccelerationZ());
  float accX_angle=atan(-1*accX/(sqrt(pow(accY,2)+pow(accZ,2))))*(180/PI);
  //Serial.println(accX_angle);
  //rotX=convertRawGyro(BMI160.getRotationX());
  float summed_acc=accX+accY+accZ;
  float grav_x_offset=sin(accX_angle*(PI/180))*9.81;
  //Serial.println(grav_x_offset);
  //Serial.println(accX+grav_x_offset);
  //Serial.println();
  if(accX>.10 || accX<-.10){
    if(false){
      Serial.print("adding \t");
      Serial.println(accX);
    }
    velocity+=(accX+err_adj);
    //Serial.println(velocity);
  }
  



//print values
  if(false){
    Serial.print("acc:\t");
    Serial.print(accX);
  // Serial.println(summed_acc);
  //Serial.print("\t");
    //Serial.print(convertRawAcc(BMI160.getAccelerationY()));
    //Serial.println(velocity);
    //Serial.print(xy_acc);
    Serial.println();
  }

  //print raw accel data
  if(false){
    Serial.println("acceleration data:");
    Serial.print("X:\t");
    Serial.println(BMI160.getAccelerationX());
    Serial.print("Y:\t");
    Serial.println(BMI160.getAccelerationY());
    Serial.print("Z:\t");
    Serial.println(BMI160.getAccelerationZ());
  }
  //gyro raw data
  if(false){
    Serial.println("gyro data:");
    Serial.print("X:\t");
    //Serial.println(rotX);
     Serial.println((int)convertRawGyro(BMI160.getRotationX()));
    // Serial.print("Y:\t");
    // Serial.println(convertRawGyro(BMI160.getRotationY()));
    // Serial.print("Z:\t");
    // Serial.println(convertRawGyro(BMI160.getRotationZ()));
  }
  
  
  delay(10);
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;
}
/*
* +/- 2g           | 8192 LSB/mg
 * +/- 4g           | 4096 LSB/mg
 * +/- 8g           | 2048 LSB/mg
 * +/- 16g          | 1024 LSB/mg
 * */
float convertRawAcc(int aRaw){
  
  return (aRaw*9.81)/4096;
}

int get_xy_magn(int x, int y){

  int result=0;
 
  result=sqrt(square(x)+square(y));
  //if x or y is less than 0, result is neg, if both are or neither are, result is pos
  result= ((x<0)^(y<0)) ?(result*(-1)):(result);
  return result;

}

//function called when brake_timer val reached
//used to get sum of acceleration measurments, reset the sum over time period, and update value
//!!NOTE add way to track previous 3-5 velocities to make sure it is truly negative or truly positive and not a one off
bool brake_timer_handler(){
  Serial.println("in brake timer handler");
  //true if velocity is neg,false if vel is pos
  bool toggle_led=false;
  Serial.println(toggle_rate);
  //if vel was previously braking,
  bool full_brake=false;
  
  //if velocity is decreasing, toggle leds and increase blink rate
  if(velocity<-3){
    toggle_led=true;
    //if toggle rate is fast enough, just brake and dont descrease value anymore
    if(toggle_rate<=100){
      full_brake=true;
    }
    else{
      full_brake=false;
      toggle_rate-=50;
    }
    
  }
  //else if moving forward, pos vel, turn off blink and all leds. And reset blink rate
  else if(velocity>3){
    toggle_led=false;
    full_brake=false;
    toggle_rate=700;
    //add check for if turn signal is on for right or left
    //if left is blining, leave dim alone
    if(turn_left){
      digitalWrite(left_bright_pin,LOW);
      digitalWrite(right_dim_pin,LOW);
      digitalWrite(right_bright_pin,LOW);
    }
    //else if right is blinking, leave right dim alone
    else if(turn_right){
      digitalWrite(left_dim_pin,LOW);
      digitalWrite(left_bright_pin,LOW);
      digitalWrite(right_bright_pin,LOW);
    }
    //else turn all leds off
    else{
      digitalWrite(left_dim_pin,LOW);
      digitalWrite(left_bright_pin,LOW);
      digitalWrite(right_dim_pin,LOW);
      digitalWrite(right_bright_pin,LOW);
    }
  }

// add check here for previous velocities and stuff
  if(toggle_led){
    //again check for turn signals, if either is on, leave respective light along
    if(turn_left)
      digitalWrite(right_dim_pin,!digitalRead(right_dim_pin));
    else if(turn_right)
      digitalWrite(left_dim_pin,!digitalRead(left_dim_pin));
    else{
      digitalWrite(left_dim_pin,!digitalRead(left_dim_pin));
      digitalWrite(right_dim_pin,!digitalRead(right_dim_pin));
    }
    
  }
  if(full_brake){
    //add check for if turn signal is on, if so then continue blinking
    if(turn_left)
      digitalWrite(right_bright_pin,HIGH);
    else if(turn_right)
      digitalWrite(left_bright_pin,HIGH);
    else{
      digitalWrite(left_bright_pin,HIGH);
      digitalWrite(right_bright_pin,HIGH);
    }
  }
  
  //schedule next time to call and update leds
  sum_acc_timer.in(toggle_rate,brake_timer_handler);
  
//return true to conitnue counter
  return false;
}

bool calc_vel(){
 // Serial.println("in toggle handler");
  displacement+=velocity;
  prev_velocity=velocity;
  velocity=0;
  return true;
}

//potentially add it so if braking, turn signal also turns on the bright led
bool turn_timer_handler(){
  //if turn left or turn right are true, then toggle respective led
  if(turn_left){
    digitalWrite(left_dim_pin,!digitalRead(left_dim_pin));
  }
  else if(turn_right){
    digitalWrite(right_dim_pin,!digitalRead(right_dim_pin));
  }

  return true;
}