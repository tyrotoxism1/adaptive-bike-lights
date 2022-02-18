/**************************************88
 * Author: Micah Janzen
 * Date: 2/3/2022
 * Program Description: This program utilized the BMI160 6 axis accelerometer and gyroscope sensor to 
 * control 2 leds. LEDs are turned on when acceleration is negative and two pushbuttons act as turn signals which
 * make the LEDs blink 
 * Included library(driver code for BMI160): https://github.com/bastian2001/BMI160-Arduino-1
*/
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
bool timer_funct();
bool toggle_handler();


auto timer = timer_create_default();
auto toggle_led_timer=timer_create_default();


/** Global Vars **/
//
int toggle_rate=1000; //start out at 1 second toggle rate 
//used to keep track of time similiar to timer
int long last_millis=0;
int long curr_millis=0;
//speed(only in x direction atm) info. used to sum the accelerometer readings over the period of 1 sec
float velocity,prev_velocity=0;



//left push button signal(active low)
int const btn_left_pin=3;
bool btn_left_val=true;
//right push button signal(active low)
int const btn_right_pin=4;
bool btn_right_val=true;
//left led signal
int const led_left_pin=5;
bool led_left_val=false;
bool blink_left=false;
//Right led signal
int const led_right_pin=6;
bool led_right_val=false;
bool blink_right=false;
//vars to attempt to keep track of general motion
bool moving_forward=false;
bool braking=false;
//dispalcement aimed to keep track of total ditance traveled
int displacement=0;
//var to try and adjust for error at resting place
const float err_adj=.06;

/*
Setup Description: Initialize Serial, set 2 leds as outuput and take pushbutton as input.
Setup BMI160 sensor, using SPI communication between sensor and arduino, and calibrate sensor.
*/
void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open
  //set btn as input, leds as ouptu
  pinMode(btn_left_pin, INPUT_PULLUP);
  pinMode(btn_right_pin, INPUT_PULLUP);
  pinMode(led_left_pin, OUTPUT);
  pinMode(led_right_pin, OUTPUT);


  // initialize device
  Serial.println("Initializing IMU device...");
  BMI160.begin(BMI160GenClass::SPI_MODE, /* SS pin# = */10);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);
  //Serial.println(BMI160.setMotionDetectionThreshold());

  // Set the accelerometer range to 250 degrees/second
  BMI160.setGyroRange(125);
  BMI160.initialize();
  BMI160.setAccelOffsetEnabled(true);
  BMI160.autoCalibrateXAccelOffset(0);
  BMI160.autoCalibrateYAccelOffset(0);
  //BMI160.autoCalibrateZAccelOffset(1);
  //BMI160.setAccelRate(8);

 // BMI160.setAccelRate(32000);
  BMI160.interruptsEnabled(CURIE_IMU_MOTION);
  //BMI160.setDetectionThreshold(CURIE_IMU_MOTION,2);
  BMI160.setFullScaleAccelRange(8);
  BMI160.setAccelRate(5);
  Serial.println(BMI160.getFullScaleAccelRange());
  //Serial.println(BMI160.getDetectionThreshold(CURIE_IMU_MOTION));
  Serial.println("Initializing IMU device...done.");
  timer.every(1000,timer_funct);
  toggle_led_timer.every(toggle_rate,toggle_handler);
}

/*
Loop Description: Reads in button values as well as the acceleration+gyroscope raw values from sensor.
if button was pressed, toggle blinking with respective led(left or right). If there is negative acceleration
Turn on led that isn't blinking and let the led that is blinking continue to blink.
*/
void loop() {
  
  float accX,accY;
  timer.tick();
  //toggle_led_timer.tick();
 

  //read button values
  btn_left_val=digitalRead(btn_left_pin);
  btn_right_val=digitalRead(btn_right_pin);


/*****  Just stuff for turn signls and button, not needed for checkoff of sensor  *****/
  /*
  //if right btn pressed(active low), blink right led
  if(btn_right_val==0){
    //flip blink val to turn toggle essentially
    blink_right=(!blink_right);
    curr_millis=millis();

    //pressing left signal should cancel other turn signal if pressed after
    if(blink_left)
      blink_left=false;
  }
  else{//default to just led on(half brightness, fog light brakes)
    analogWrite(led_right_pin,125);
  }
  //active low
  if(btn_left_val==0){
    //flip blink val to turn toggle essentially
    blink_left=(!blink_left);
    curr_millis=millis();

    //pressing left signal should cancel other turn signal if pressed after
    if(blink_right)
      blink_right=false;
  }
  else{ //default to just led on(half brightness, fog light brakes)
    analogWrite(led_left_pin, 125);
  }
  */

/*
  if(blink_right){
    analogWrite(led_right_pin,(led_right_val?255:0));
    led_right_val=(!led_right_val);
  }
  else if(blink_left){
    digitalWrite(led_left_pin,(led_left_val?255:0));
    led_left_val=(!led_left_val);
  }
  */


  //add new val to arr of ma for X Y and Z arrs
  //update_arr(acc_maX, (convertRawAcc(BMI160.getAccelerationX())) );
  accX=convertRawAcc(BMI160.getAccelerationX());
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
    Serial.print(convertRawAcc(BMI160.getAccelerationX()));
    Serial.print("\t");
    //Serial.print(convertRawAcc(BMI160.getAccelerationY()));
    Serial.println(velocity);
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
    Serial.println(BMI160.getRotationX());
    Serial.print("Y:\t");
    Serial.println(BMI160.getRotationY());
    Serial.print("Z:\t");
    Serial.println(BMI160.getRotationZ());
  }
  
  
  delay(150);
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

//function called when timer val reached
//used to get sum of acceleration measurments, reset the sum over time period, and update value
bool timer_funct(){
  displacement+=velocity;
  prev_velocity=velocity;
  //if velocity is decreasing, or standing still, "brakes" are on
  if(velocity<0){
    digitalWrite(led_left_pin,HIGH);
    digitalWrite(led_right_pin,HIGH);
  }
  //else toggle leds to flash
  else{
    digitalWrite(led_left_pin,!digitalRead(led_left_pin));
    digitalWrite(led_right_pin,!digitalRead(led_right_pin));
  }
  //just for printing and testing vals
  if(true){
    Serial.print("summed accel: \t");
    Serial.print(velocity);
    Serial.print("\tDisplacement:\t");
    Serial.println(displacement);
  }
  
  velocity=0;
//return true to conitnue counter
  return true;
}

bool toggle_handler(){
  Serial.println("in toggle handler");
  toggle_rate+=10;
  Serial.print("left led: \t");
  Serial.println(led_left_val);
  Serial.print("Rigth led: \t");
  Serial.println(led_right_val);
  digitalWrite(led_left_pin,led_left_val);
  led_left_val=(!led_left_val);
  digitalWrite(led_right_pin,led_right_val);
  led_right_val=(!led_right_val);
  //toggle_led_timer.every(toggle_rate,toggle_handler);
  return true;
}