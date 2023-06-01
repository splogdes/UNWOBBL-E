#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <cmath>
#include "Classes.h"

#define SPR 400.0
#define dirPinR 12
#define stepPinR 14
#define dirPinL 27
#define stepPinL 26
#define Ts_outer 0.04
#define Ts_inner 0.004

struct dir{
  double x;
  double y;
  double z;
};

void core0( void * pvParameters );
void core1( void * pvParameters );

SemaphoreHandle_t i2cSemaphore;
void createSemaphore(){
    i2cSemaphore = xSemaphoreCreateMutex();
    xSemaphoreGive( ( i2cSemaphore) );
}

// Lock the variable indefinietly. ( wait for it to be accessible )
void lockVariable(){
    xSemaphoreTake(i2cSemaphore, 100);
}

// give back the semaphore.
void unlockVariable(){
    xSemaphoreGive(i2cSemaphore);
}

TaskHandle_t Task1;
TaskHandle_t Task2;

double angle_acc_d = 0;

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
  xTaskCreatePinnedToCore(core0,"Task1",100000,NULL,1,&Task1,0);
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(core1,"Task2",100000,NULL,1,&Task2,1);
  delay(500); 

}


void core0( void * pvParameters ){
  Serial.println(xPortGetCoreID());
  Adafruit_MPU6050 mpu;

  //PID angle_rate_controller(0.5,0.5,0,Ts_inner);
  PID angle_controller(1,1,0,Ts_inner);
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
  //angle_rate_controller.init();
  angle_controller.print_coefficients();
  //angle_rate_controller.print_coefficients();

  for(;;){

    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    unsigned long start_time = micros();
    double pitch = atan2(a.acceleration.y,sqrt(a.acceleration.x*a.acceleration.x+a.acceleration.z*a.acceleration.z));
    pitch = pitch_filter.filter(g.gyro.x - bias.x,pitch, Ts_inner);
    /* Angle Controler Desired angle 0 */
    double tmp = angle_controller.filter(pitch - 0);
    //double tmp = angle_rate_controller.filter(theta_rate_d - (g.gyro.x - bias.x));
    //lockVariable();
    angle_acc_d = tmp; //change value of the shared angle_acc_d
    //unlockVariable();

    unsigned long stop_time = micros();
    if((stop_time - start_time)>1000000*Ts_inner){
      while(1){
        Serial.println("Warning controller takes longer than Ts_inner!");
        delay(1000);
      }
    }

    delayMicroseconds(1000000*Ts_inner - (stop_time - start_time));

  } 
}


void core1( void * pvParameters ){
  Serial.println(xPortGetCoreID());
  int next = 1;
  double t = 1000;
  double w , d;
  double step = 2*(PI/SPR);
  bool direction = 1;
  digitalWrite(dirPinR, direction);
  digitalWrite(dirPinL, !direction);
  double acc_d;

  for(;;){

    //lockVariable();
    acc_d = angle_acc_d; // read shared value 
    //unlockVariable();
    double time = Ts_outer;
    while (abs(acc_d) < 0.001)
    {
      delay(0.04);
      acc_d = angle_acc_d; // read shared value6
    }
    Serial.println(acc_d);

    while(time>0){
      double w0 = step/t;
      /* change direction */
      if(w0*w0 + 2*acc_d*step < 0){
        step = -step;
        direction = !direction;
        digitalWrite(dirPinR, direction);
        digitalWrite(dirPinL, !direction);
      }
      /* work out the time between steps to achieve the correct acceleration */
      w =  sqrtf(w0*w0 + 2*acc_d*step);
      t = (w - w0)/acc_d;
      /* step the motors */
      d = (step > 0 ? 1 : -1)*t;
      time -= d;
      if(time > 0){
        delayMicroseconds(1000000*d);
        digitalWrite(stepPinR, next);
        digitalWrite(stepPinL, next);
        next = !next;
      } 
    }
  }
}

void loop() {
  
}