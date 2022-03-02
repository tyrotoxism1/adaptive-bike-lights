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



/********************* Functions **********************8**/
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
//function handlers for timers
bool brake_timer_handler();
bool calc_vel();
bool turn_timer_handler();



//create timers and bind fucntion handlers to call when timer is up
auto brake_timer = timer_create_default();
auto sum_acc_timer=timer_create_default();
auto turn_timer=timer_create_default();


/*************** Global Vars ***********************/
//amount of time to turn on and off leds when flashing
int toggle_rate=500; 
//used to keep track of time similiar to timer
int long previous_time=0;
int long current_time=0;
int long elapsed_time=0;
//speed(only in x direction atm) info. holds  sum of the accelerometer readings over the period of 1 sec
float velocity,prev_velocity=0;
//dispalcement aimed to keep track of total ditance traveled
int displacement=0;

//2 leds, brightness controlled with pwm and the val
//LEFT LED: D5
int left_led_pin=5;
//RIGHT LED: D6
int right_led_pin=6;
int led_brightness=0;


//2 pushbuttons to trigger turn signal
//LEFT BUTTON: D9
int left_btn_pin=9;
bool left_btn_val=false;
bool turn_left=false;
//RIGHT BUTTON D10:
int right_btn_pin=10;
bool right_btn_val=false;
bool turn_right=false;
int brightness;



//var to try and adjust for error at resting place
const float err_adj=.06;
float rotX=0;
float rot_sum_x=0;
float gyro_angleX;

/*
Setup Description: Initialize Serial, set 4 led signals as outuput and take pushbutton as input.
Setup BMI160 sensor, using SPI communication between sensor and arduino, and calibrate sensor.
*/
void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  pinMode(left_led_pin, OUTPUT);
  pinMode(right_led_pin, OUTPUT);
  pinMode(left_btn_pin, INPUT_PULLUP);
  pinMode(right_btn_pin, INPUT_PULLUP);


  // initialize device
  Serial.println("Initializing IMU device...");
  BMI160.begin(BMI160GenClass::I2C_MODE, 0x68);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

  // Set the accelerometer range to 250 degrees/second
  //BMI160.setGyroRange(3);
  BMI160.initialize();
  BMI160.setFullScaleGyroRange(3);
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

  digitalWrite(7,!digitalRead(7));
 

  //read button values( ACTIVE LOW)
  left_btn_val=digitalRead(left_btn_pin);
  right_btn_val=digitalRead(right_btn_pin);

/*****************
 * BUTTON POLLING
 * ***************/

//if left button found true, swap turn signal val for left, essentially toggling the turn signal
  if(left_btn_val==0){
    turn_left=(!turn_left);
    if(turn_right==true){
      analogWrite(right_led_pin,0);
      turn_right=false;
    }
    analogWrite(left_led_pin,0);
  }
  if(right_btn_val==0){
    turn_right=(!turn_right);
    if(turn_left==true){
      digitalWrite(left_led_pin,0);
      turn_left=false;
    }
    digitalWrite(right_led_pin,0); 
  }


/********************************************
 * ACC + ROT DATA RETREIVAL & MANIPULATION
 * ******************************************/
//get rotation for X and acceleration of each axis
  rotX=convertRawGyro(BMI160.getRotationY());
  //try to not account for small noise movements
  if(rotX<.3 && rotX>-.3){
    rotX=0;
  }
  accX=convertRawAcc(BMI160.getAccelerationX());
  accY=convertRawAcc(BMI160.getAccelerationY());
  accZ=convertRawAcc(BMI160.getAccelerationZ());
   //adjusts X acceleration data error
  if(accX>.1){
    accX+=.6;
  }
  else if(accX<-.1){
    accX+=.7;
  }
  //adjusts Y acceleration data error
   if(accY>.1){
    accY+=.25;
  }
  else if(accY<-.1){
    accY+=.35;
  }
  //adjust Z acceleratoin data error
  if(accZ>.1){
    accZ+=.3;
  }
  else if(accZ<-.1){
    accZ+=.5;
  }
  float acc_magn=(sqrt(pow(accX,2)+pow(accY,2)+pow(accZ,2)))-9.8;

  if(acc_magn>.2 || acc_magn<-.2){
    //Serial.println(acc_magn);
  }


//https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
//good amount of these calculations are referenecing this project linked above
  previous_time=current_time;
  current_time=millis();
  elapsed_time=(current_time-prev_velocity)/1000;
  gyro_angleX+=rotX*elapsed_time;
  //Serial.println(gyro_angleX);
  
  
  

 
  

 //finds the angle from the x axis(i think essentially like the angle around the y axis), and converts to degrees
  float accX_angle=atan(-1*accX/(sqrt(pow(accY,2)+pow(accZ,2))))*(180/PI);
  
  //float pitch=.96*gyro_angleX+.04*accX_angle;
  //Serial.println(accX_angle);
  //rotX=convertRawGyro(BMI160.getRotationX());
  //takes the angle from x axis, converting the angle to radians, then multiplies by gravity component affecting x axis
  float grav_x_offset=sin(accX_angle*(PI/180))*9.81;
  //Serial.println(grav_x_offset);



  //if acceleration is not noise, sum over period to get velocity
  if(accX>.1 || accX<-.1){
    if(false){
      Serial.print("adding \t");
      Serial.println(acc_magn);
    }
    //since acc_magn is magninuted, we need to get if accel is neg or not
    if(accX<-.05){
      acc_magn*=-1;
    }

    velocity+=(accX);
    //Serial.println(velocity);
  }
  
/****************************************
 * TEST PRINTING
 * **************************************/
if(false){
  Serial.println(velocity);
//print val for the angle from x axis
  //Serial.println(accX_angle);
//print val for acceleration with the gravity offset added
  //Serial.println(accX+grav_x_offset);

}

if(true){
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


//NOTE: cant truly get velocity, so to determine braking just use acceleration and decceleration
//for turning off the turn signal after turn, just detect forward motion and then set a timer

//function called when brake_timer val reached
//used to get sum of acceleration measurments, reset the sum over time period, and update value
//!!NOTE add way to track previous 3-5 velocities to make sure it is truly negative or truly positive and not a one off
bool brake_timer_handler(){
  
  //true if velocity is neg,false if vel is pos
  bool toggle_led=false;
  //true if the toggle_rate reaches max falsh rate, false otherwise
  bool full_brake=false;
  
  //if velocity is decreasing, toggle leds and increase blink rate
  if(velocity<-3){
    toggle_led=true;
    //if toggle rate is fast enough, just brake and dont descrease value anymore
    if(toggle_rate<=30){
      full_brake=true;
      led_brightness=255;
    }
    else{
      full_brake=false;
      toggle_rate-=50;
      led_brightness+=20;
    }
  }
  
  //else if moving forward, pos vel, turn off blink and all leds. And reset blink rate
  else if(velocity>1){
    toggle_led=false;
    full_brake=false;
    //led_brightness=100;
    toggle_rate=500;
    //add check for if turn signal is on for right or left
    //if left is blining, leave dim alone
    if(turn_left){
      analogWrite(right_led_pin,LOW);
    }
    //else if right is blinking, leave right dim alone
    else if(turn_right){
      analogWrite(left_led_pin,LOW);
    }
    //else turn all leds off
    else{
      analogWrite(left_led_pin,LOW);
      analogWrite(right_led_pin,LOW);
    }
  }

  if(toggle_led){
    //again check for turn signals, if either is on, leave respective light along
    if(turn_left){
      if(brightness!=0)
        brightness=0;
      else
        brightness=led_brightness;
      analogWrite(right_led_pin,brightness);
    }
    else if(turn_right){
      if(brightness!=0)
        brightness=0;
      else
        brightness=led_brightness;
      analogWrite(left_led_pin,brightness);
    }
    else{
      if(brightness==0)
        brightness=led_brightness;
      else
        brightness=0;
      //analogWrite(left_led_pin,LOW);
      analogWrite(left_led_pin,brightness);
      analogWrite(right_led_pin, brightness);
    }
    
  }
  if(full_brake){
    led_brightness=0;
    brightness=0;
    //add check for if turn signal is on, if so then continue blinking
    if(turn_left)
      analogWrite(right_led_pin,255);
    else if(turn_right)
      analogWrite(left_led_pin,255);
    else{
      analogWrite(left_led_pin,255);
      analogWrite(right_led_pin,255);
    }
  }
  
  
  //schedule next time to call and update leds
  sum_acc_timer.in(toggle_rate,brake_timer_handler);
  
//return false to stop timer, but since we created a new timer "time" above, it will be called in toggle rate amt of time
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
    digitalWrite(left_led_pin,!digitalRead(left_led_pin));
  }
  else if(turn_right){
    digitalWrite(right_led_pin,!digitalRead(right_led_pin));
  }

  return true;
}


