#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <cmath>
#include "Classes.h"


// 0.1 4 30 -0.004 -0.003 0 30 140 12 0.1

#define SPR 6400.0
#define dirPinR 25
#define stepPinR 33
#define dirPinL 27
#define stepPinL 26
#define Ts_inner 0.01
#define Dw 0.067


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
  double kp_theta = 30;
  double angle_sat = 100; 
  PID velocity_controller(0,0,0,angle_sat);
  double kp_pos = 0;
  double sat_velocity = 0.02;
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
  double akp,aki,akd,vkp,vki,vkd;
  angle_rate_controller.init();
  pitch_filter.init();
  double pos_d = 0.2;
  for(;;){
    unsigned long start_time = micros();
    if(Serial.available()){
      Serial.println("");
      angle_acc_d = 0;
      position = 0;
      rst = 1;
      angle_sat = Serial.parseFloat();
      sat_velocity = Serial.parseFloat();
      kp_pos = Serial.parseFloat();
      vkp = Serial.parseFloat();
      vki = Serial.parseFloat();
      vkd = Serial.parseFloat();
      kp_theta = Serial.parseFloat();
      akp = Serial.parseFloat();
      aki = Serial.parseFloat();
      akd = Serial.parseFloat();
      velocity_controller.set_coefficients(vkp,vki,vkd,angle_sat);
      angle_rate_controller.set_coefficients(akp,aki,akd);
      velocity_controller.init();
      angle_rate_controller.init();
      pitch_filter.init();
      Serial.print("position Kp_pos:");Serial.print(kp_pos*1000000);
      velocity_controller.print_coefficients();
      Serial.print("angle Kp_theta:");Serial.print(kp_theta);
      angle_rate_controller.print_coefficients();
    }
    count++;
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    double pitch = atan2(a.acceleration.z,sqrt(a.acceleration.x*a.acceleration.x+a.acceleration.y*a.acceleration.y));
    double kpitch = pitch_filter.filter(g.gyro.y - bias.y,pitch);
    /* Angle Controler Desired angle 0 */
    double velocity_d = kp_pos*(pos_d - PI*Dw*double(position)/SPR);
    if(sat_velocity < fabs(velocity_d)){
      if(velocity_d>0){
        velocity_d = sat_velocity;
      }else{
        velocity_d = -sat_velocity;
      }
    }
    double pitch_d = velocity_controller.filter(velocity_d - speed);
    double angle_rate_d = kp_theta*(pitch_d-(kpitch-0.447));
    angle_acc_d = angle_rate_controller.filter(angle_rate_d - (g.gyro.y - bias.y)); //change value of the shared angle_acc_d inner loop 
    unsigned long stop_time = micros();
    if (count %25 == 0){Serial.print("pos_d:");Serial.print(pos_d);Serial.print(" velocity_d:");Serial.print(velocity_d);Serial.print(" pitch_d:");Serial.print(pitch_d);Serial.print(" position:");Serial.print(PI*Dw*double(position)/SPR);Serial.print(" kpitch:");Serial.print(57.3*(kpitch-0.447));Serial.print(" acc:");Serial.print(angle_acc_d);Serial.print(" speed:");Serial.println(speed);}
    if (count % 4000 == 0) pos_d *= -1;
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