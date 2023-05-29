#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <cmath>
#include "Classes.h"

#define SPR 200
#define dirPinR 12
#define stepPinR 14
#define dirPinL 27
#define stepPinL 26
#define Ts_outer 1/25
#define Ts_inner 1/250

struct dir{
  float x;
  float y;
  float z;
};

SemaphoreHandle_t i2cSemaphore;
void createSemaphore(){
    i2cSemaphore = xSemaphoreCreateMutex();
    xSemaphoreGive( ( i2cSemaphore) );
}

// Lock the variable indefinietly. ( wait for it to be accessible )
void lockVariable(){
    xSemaphoreTake(i2cSemaphore, portMAX_DELAY);
}

// give back the semaphore.
void unlockVariable(){
    xSemaphoreGive(i2cSemaphore);
}

TaskHandle_t Task1;
TaskHandle_t Task2;

volatile double angle_acc_d = 0;

/* To bias the gyro scope */
dir bias_gyro(Adafruit_MPU6050 * mpu){
  sensors_event_t a, g, temp;
  dir store = {0,0,0};
  for(int i=0;i<100;i++){
    (*mpu).getEvent(&a, &g, &temp);
    store.x = g.gyro.x;
    store.y = g.gyro.y;
    store.z = g.gyro.z;
  }
  return store;
}

void setup() {

  /* For Wheels */
  pinMode(stepPinR, OUTPUT);
  pinMode(dirPinR, OUTPUT);
  pinMode(stepPinL, OUTPUT);
  pinMode(dirPinL, OUTPUT);

  Serial.begin(115200);
  while(!Serial)

  createSemaphore();
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(core0,"Task1",10000,NULL,1,&Task1,0);
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(core1,"Task2",10000,NULL,1,&Task2,1);
  delay(500); 

}


void core0( void * pvParameters ){
  Adafruit_MPU6050 mpu;

  PID angle_rate_controller(24.3136228259616,44.265308660561,0.048450184417281,Ts_inner);
  PID angle_controller(6.79854249968418,8.48242460792782,0.519954888426499,Ts_inner);
  Kalman_filter_pitch pitch_filter(0.001,0.003,0.03);

  dir bias;

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  bias = bias_gyro(&mpu);

  angle_controller.init();
  angle_rate_controller.init();
  angle_controller.print_coefficients();
  angle_rate_controller.print_coefficients();

  for(;;){

    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    unsigned long start_time = millis();
    double pitch = atan2(a.acceleration.y,sqrt(a.acceleration.x*a.acceleration.x+a.acceleration.z*a.acceleration.z));
    pitch = pitch_filter.filter(g.gyro.y - bias.y,pitch,Ts_inner);

    /* Angle Controler Desired angle 0 */
    double theta_rate_d = angle_controller.filter(pitch - 0);
    double tmp = angle_rate_controller.filter(theta_rate_d - (g.gyro.y - bias.y));

    lockVariable();
    angle_acc_d = tmp; //change value of the shared angle_acc_d
    unlockVariable();

    unsigned long stop_time = millis();
    if(1000000*(start_time - stop_time)>Ts_inner){
      while(1){
        Serial.println("Warning controller takes longer than Ts_inner!");
        delay(1000);
      }
    }

    delay(Ts_inner/1000 - 1000*(start_time - stop_time));

  } 
}


void core1( void * pvParameters ){

  float t , w = 0, acc_d, w0;
  float step = 2*PI/SPR;
  bool direction = HIGH;
  // Set the spinning direction counterclockwise:
  digitalWrite(dirPinR, direction);
  digitalWrite(dirPinL, direction);

  for(;;){

    lockVariable();
    acc_d = angle_acc_d; // read shared value 
    unlockVariable();

    float time = Ts_outer;

    while(time>0){

      /* change direction */
      if(w*w + 2*acc_d*step < 0){
        step = -step;
        direction = !direction;
        digitalWrite(dirPinR, direction);
        digitalWrite(dirPinL, direction);
      }
      /* work out the time between steps to achieve the correct acceleration */
      w0 = w; 
      w = sqrtf(w*w + 2*acc_d*step);
      t = (w - w0)/acc_d;
      time -= t;
      /* step the motors */
      if(time > 0){
        delayMicroseconds(500000*t);
        digitalWrite(stepPinR, HIGH);
        digitalWrite(stepPinL, HIGH);

        delayMicroseconds(500000/t);
        digitalWrite(stepPinR, LOW);
        digitalWrite(stepPinL, LOW);
      }
    }
  }
}

void loop() {
  
}