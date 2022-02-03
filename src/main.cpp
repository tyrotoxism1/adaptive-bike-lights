#include <Arduino.h>

#include <BMI160-Arduino/BMI160.h>
#include <BMI160-Arduino/BMI160Gen.h>
#include <BMI160-Arduino/CurieIMU.h>

/** Functions **/
float convertRawGyro(int gRaw);
float convertRawAcc(int aRaw);
int get_xy_magn(int x, int y);
//takes in array, then returns the average
int get_ma(int* arr);
//takes in arr, shifts each value "left" discarding [0] index, and putting new val at index 20
void update_arr(int* arr,int new_val);

/** Global Vars **/
//defines the arr size of the moving avg for acc and rot if needed
const int avg_arr_size=5;
//used to keep track of time similiar to timer
int long last_millis=0;
int long curr_millis=0;


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
  BMI160.autoCalibrateZAccelOffset(1);
  //BMI160.setAccelRate(8);

 // BMI160.setAccelRate(32000);
  BMI160.interruptsEnabled(CURIE_IMU_MOTION);
  BMI160.setDetectionThreshold(CURIE_IMU_MOTION,2);
  Serial.println(BMI160.getDetectionThreshold(CURIE_IMU_MOTION));
  Serial.println("Initializing IMU device...done.");
}

void loop() {
  int accX_ma,accY_ma,accZ_ma,xy_acc, rotX_ma, rotY_ma, rotZ_ma;

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
  /*
  //timer used to turn off turn signal after 5 seconds, doesnt quite work atm
  if((curr_millis-last_millis>5000) && (blink_right||blink_left)){ //if blinking turn signal is on for certain time, reset to default
    Serial.println("Turn signal reset");
    blink_left=false;
    blink_right=false;
    last_millis=curr_millis;
  }
  */


  //add new val to arr of ma for X Y and Z arrs
  update_arr(acc_maX, (convertRawAcc(BMI160.getAccelerationX())) );
  accX_ma=get_ma(acc_maX);
  //accX_ma=convertRawAcc(BMI160.getAccelerationX());

  update_arr(acc_maY, (convertRawAcc(BMI160.getAccelerationY())) );
  accY_ma=get_ma(acc_maY);
  //accY_ma=convertRawAcc(BMI160.getAccelerationY());

  update_arr(acc_maZ, (convertRawAcc(BMI160.getAccelerationZ())) );
  accZ_ma=get_ma(acc_maZ);
  //accZ_ma=convertRawAcc(BMI160.getAccelerationZ());


  xy_acc=get_xy_magn(accX_ma,accY_ma);

  update_arr(rot_maX, (convertRawGyro(BMI160.getRotationX())) );
  rotX_ma=get_ma(rot_maX);
  
  update_arr(rot_maY, (convertRawGyro(BMI160.getRotationY())) );
  rotY_ma=get_ma(rot_maY);

  update_arr(rot_maZ, (convertRawGyro(BMI160.getRotationZ())) );
  rotZ_ma=get_ma(rot_maZ);

  //if val is not from noise and is actual movement, then add to vel
  //idea doens't really work atm
  if(accX_ma>1 || accX_ma<-1){
    velocityX+=accX_ma;
    }
  if(accY_ma>1 || accY_ma<-1){
      velocityY+=accY_ma;
    }

    //if deccelerating, and not noise brakes on
    if(xy_acc<-1){
      if((blink_left==false)&& (blink_right==false)){
        digitalWrite(led_left_pin,HIGH);
        digitalWrite(led_right_pin,HIGH);
      }
      //if blinking, continue to blink but turn other led on as well
      else if(blink_left)
        digitalWrite(led_right_pin, HIGH);
      else if(blink_right)
        digitalWrite(led_left_pin,HIGH);
      //hold brake light for a bit afterwards(replace with timer eventually)
      delay(200);
      
    }
 

//below if statement doesn't ever return true even if sensor is moved quickly.
//(BMI160.motionDetected(X_AXIS,POSITIVE)) || (BMI160.motionDetected(Y_AXIS,POSITIVE))
//print acceleration values
  if(true){
  Serial.print("acc:\t");
  Serial.print(accX_ma);
  Serial.print("\t");
  Serial.print(accY_ma);
  Serial.print("\t");
  Serial.print(xy_acc);
  
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
  //maybe test that this is m/s^2 by using video and tracking sensor similar to physics lab, and compare results
  //9.82 for grav, 1000 for mg, 8192 for +-2g.
  return (aRaw*9.82*(10))/8192;
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