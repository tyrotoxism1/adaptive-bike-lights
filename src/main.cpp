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


auto timer = timer_create_default();

/** Global Vars **/
//defines the arr size of the moving avg for acc and rot if needed
const int avg_arr_size=5;
//used to keep track of time similiar to timer
int long last_millis=0;
int long curr_millis=0;
//accel info
float accel_sum,prev_accel_sum=0;


//used to keep track of moving average for X acceleration
int acc_maX[avg_arr_size]={0};
int acc_maY[avg_arr_size]={0};
int acc_maZ[avg_arr_size]={0};
//moving average of each axis of rotation
int rot_maX[avg_arr_size]={0};
int rot_maY[avg_arr_size]={0};
int rot_maZ[avg_arr_size]={0};
//left push button signal(active low)
int const btn_left_pin=11;
bool btn_left_val=true;
//right push button signal(active low)
int const btn_right_pin=12;
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

//keep track of velocity by adding acceleration values above threshold
//idea was to try and sum up acceleration points when motion happens, however doesn't work since valeus don't exactly
//add to 0, so would be able to really tell if stopped or how fast person is going.
int velocityX=0;
int velocityY=0;

/*
Setup Description: Initialize Serial, set 2 leds as outuput and take pushbutton as input.
Setup BMI160 sensor, using SPI communication between sensor and arduino, and calibrate sensor.
*/
void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open
  //set btn as input, leds as ouptu
  pinMode(btn_left_pin, INPUT);
  pinMode(btn_right_pin, INPUT);
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
  Serial.println(BMI160.getFullScaleAccelRange());
  //Serial.println(BMI160.getDetectionThreshold(CURIE_IMU_MOTION));
  Serial.println("Initializing IMU device...done.");
  timer.every(1000,timer_funct);
}

/*
Loop Description: Reads in button values as well as the acceleration+gyroscope raw values from sensor.
if button was pressed, toggle blinking with respective led(left or right). If there is negative acceleration
Turn on led that isn't blinking and let the led that is blinking continue to blink.
*/
void loop() {
  
  float accX,accY,accX_ma,accY_ma,accZ_ma,xy_acc, rotX_ma, rotY_ma, rotZ_ma;
  
  timer.tick();
 

  //read button values
  btn_left_val=digitalRead(btn_left_pin);
  btn_right_val=digitalRead(btn_right_pin);
  
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

  if(blink_right){
    analogWrite(led_right_pin,(led_right_val?255:0));
    led_right_val=(!led_right_val);
  }
  else if(blink_left){
    digitalWrite(led_left_pin,(led_left_val?255:0));
    led_left_val=(!led_left_val);
  }


  //add new val to arr of ma for X Y and Z arrs
  //update_arr(acc_maX, (convertRawAcc(BMI160.getAccelerationX())) );
  accX=convertRawAcc(BMI160.getAccelerationX());
  if(accX>.05 || accX<-.05){
    Serial.print("adding \t");
    Serial.println(accX);
    
    accel_sum+=accX;
    //Serial.println(accel_sum);
  }
  //accX_ma=convertRawAcc(BMI160.getAccelerationX());


 

    
 

//below if statement doesn't ever return true even if sensor is moved quickly.
//(BMI160.motionDetected(X_AXIS,POSITIVE)) || (BMI160.motionDetected(Y_AXIS,POSITIVE))
//print acceleration values
  if(false){
    Serial.print("acc:\t");
    Serial.print(convertRawAcc(BMI160.getAccelerationX()));
    Serial.print("\t");
    //Serial.print(convertRawAcc(BMI160.getAccelerationY()));
    Serial.println(accel_sum);
    //Serial.print(xy_acc);
    
    Serial.println();
  }
  //print test velocity values
  if(false){
    Serial.print("velocity: \t");
    Serial.print(velocityX);
    Serial.print("\t");
    Serial.print(velocityX);
    Serial.println();
  }
  //print gryo rotation values
  if(false){
    //display tab-separated gyro x/y/z values
    Serial.print("rotation:\t");
    Serial.print((convertRawGyro(BMI160.getRotationX())));
    Serial.print("\t");
    Serial.print((convertRawGyro(BMI160.getRotationY())));
    Serial.print("\t");
    Serial.print((convertRawGyro(BMI160.getRotationZ())));
    Serial.print("\t");
  }
  //print button val stuff
  if(false){
    Serial.print("button val(R L): \t");
    Serial.print(btn_right_val);
    Serial.print("\t");
    Serial.print(btn_left_val);
    Serial.println();
  }
  delay(1);
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
  
  return (aRaw*9.81)/16384;
}

int get_ma(int* arr){
  int avg=0;
  //for loops run for 20 since we know size of arrs
  for(int i=0; i<avg_arr_size; i++){
    avg+=arr[i];
  }
  //divide to get average
  avg/=avg_arr_size;

  return avg;
}

void update_arr(int* arr, int new_val){
  //first go from the beg of arr, replacing the prev  index with the next index val
  for(int i=0; i<avg_arr_size-1; i++){ //should only need to go to 18 index since last is replaced with new_val
    arr[i]=arr[i+1];
  }
  arr[avg_arr_size-1]=new_val;
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
  if(false)
    Serial.print("summed accel: \t");
    Serial.println(accel_sum);
  prev_accel_sum=accel_sum;
  accel_sum=0;
//return true to conitnue counter
  return true;
}