#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <cmath>
#include "Classes.h"

#define SPR 6400.0
#define dirPinR 12
#define stepPinR 14
#define dirPinL 27
#define stepPinL 26
#define Ts_inner 0.01

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

TaskHandle_t Task1;
TaskHandle_t Task2;

double angle_acc_d = 0;
double speed = 0;
int position = 0;
bool rst = 0;

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

  Adafruit_MPU6050 mpu;

  PID angle_rate_controller(100,10,0);
  double kp_theta = 20;
  PID velocity_controller(1,1,0);
  double kp_pos = 1;
  Kalman_filter_pitch pitch_filter(0.001,0.003,0.03);

  dir bias;

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);

  bias = bias_gyro(&mpu);

  int count = 0;
  double nkp,nki,nkd;
  angle_rate_controller.init();
  pitch_filter.init();
  for(;;){
    unsigned long start_time = micros();
    if(Serial.available()){
      angle_acc_d = 0;
      rst = 1;
      kp_pos = Serial.parseFloat();
      nkp = Serial.parseFloat();
      nki = Serial.parseFloat();
      nkd = Serial.parseFloat();
      Serial.print("Kp_pos:");Serial.print(kp_pos);
      velocity_controller.set_coefficients(nkp,nki,nkd);
      velocity_controller.init();
      angle_rate_controller.init();
      pitch_filter.init();
      angle_rate_controller.print_coefficients();
      Serial.print("Kp_theta:");Serial.print(kp_theta);
      velocity_controller.print_coefficients();
    }
    count++;
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    double pitch = atan2(a.acceleration.z,sqrt(a.acceleration.x*a.acceleration.x+a.acceleration.y*a.acceleration.y));
    double kpitch = pitch_filter.filter(g.gyro.y - bias.y,pitch);
    /* Angle Controler Desired angle 0 */
    if (count %25 == 0){Serial.print("position:");Serial.print(position);Serial.print(" kpitch:");Serial.print(57.3*(kpitch-0.42));Serial.print(" acc:");Serial.print(angle_acc_d);Serial.print(" speed:");Serial.println(speed);}
    double tmp = kp_pos*(0 - position);
    tmp = velocity_controller.filter(tmp - speed);
    tmp = kp_theta*(0-(kpitch-0.42));
    angle_acc_d = angle_rate_controller.filter(tmp - (g.gyro.y - bias.y)); //change value of the shared angle_acc_d inner loop 
    unsigned long stop_time = micros();
    //Serial.println(stop_time - start_time);
  } 
}


void core1( void * pvParameters ){
  int next = 1,count = 0;
  double t = 1000;
  double w , d;
  double step = 2*(PI/SPR);
  bool direction = 1;
  double acc_d;

  digitalWrite(dirPinR, direction);
  digitalWrite(dirPinL, !direction);

  for(;;){
    count++;
    unsigned long start_time = micros();
    double w0 = step/t;
    speed = (step > 0 ? 1 : -1)*w0;
    while(1){
      acc_d = angle_acc_d; // read shared value 
      if(w0*w0 + 2*acc_d*step < 0){
        step = -step;
        direction = !direction;
        digitalWrite(dirPinR, direction);
        digitalWrite(dirPinL, !direction);
      }
      w =  sqrt(w0*w0 + 2*acc_d*step);
      if(abs(acc_d)>0.0000001) t = (w - w0)/acc_d;
      d = (step > 0 ? 1 : -1)*t*1000000;
      unsigned long current_time = micros();
      if((current_time - start_time) >= d){
        position += (step > 0 ? 1 : -1);
        digitalWrite(stepPinR, next);
        digitalWrite(stepPinL, next);
        next = !next;
        break;
      }
    }
    if(rst){
      rst = 0;
      int next = 1,count = 0;
      double t = 1000;
      double w , d;
      double step = 2*(PI/SPR);
      bool direction = 1;
    }
  }
}

void loop() {
  
}