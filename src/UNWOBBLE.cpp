#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <cmath>
#include "Classes.h"

#define SPR 6400.0
#define RdirPin 25
#define RstepPin 33
#define LdirPin 27
#define LstepPin 26
#define Ts_inner 0.01
#define Dw 0.0665
#define Dr 0.14

// 0.1 4 30 -0.004 -0.003 0 30 140 12 0.1

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
double Lspeed = 0, Rspeed = 0;
double acc_yaw_d = 0;
int Lposition = 0,Rposition = 0;
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
  pinMode(RstepPin, OUTPUT);
  pinMode(RdirPin, OUTPUT);
  pinMode(LstepPin, OUTPUT);
  pinMode(LdirPin, OUTPUT);

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

  PID angle_rate_controller(140,12,0.1);
  double kp_theta = 30;
  double angle_sat = 0.07; 
  PID velocity_controller(-0.004,-0.003,0,angle_sat);
  double kp_pos = 30;
  double sat_velocity = 2;
  double kp_yaw = 200;
  double yaw_rate_sat = 0.5;
  PID yaw_controller(150,15,0.1);
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
  velocity_controller.init();
  pitch_filter.init();
  yaw_controller.init();
  double pos_d = 0;
  double goal = 0;
  for(;;){
    unsigned long start_time = micros();
    if(Serial.available()){
      Serial.println("");
      yaw_rate_sat = Serial.parseFloat();
      kp_yaw = Serial.parseFloat();
      vkp = Serial.parseFloat();
      vki = Serial.parseFloat();
      vkd = Serial.parseFloat();
      yaw_controller.set_coefficients(vkp,vki,vkd);
      Serial.print("Yaw Kp_yaw:");Serial.print(kp_yaw);
      yaw_controller.print_coefficients();
      angle_acc_d = 0;
      Lposition = 0;
      Rposition = 0;
      pitch_filter.init();
      velocity_controller.init();
      angle_rate_controller.init();
      rst = 1;
    }
    count++;
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    double pitch = atan2(a.acceleration.z,sqrt(a.acceleration.x*a.acceleration.x+a.acceleration.y*a.acceleration.y));
    double kpitch = pitch_filter.filter(g.gyro.y - bias.y,pitch);
    /* Control System for balance */
    double position = Dw*PI*double(Lposition+Rposition)/(2*SPR);
    double velocity_d = kp_pos*(pos_d - position);
    if(sat_velocity < fabs(velocity_d)){
      if(velocity_d>0){
        velocity_d = sat_velocity;
      }else{
        velocity_d = -sat_velocity;
      }
    }
    double pitch_d = velocity_controller.filter(velocity_d - (Lspeed+Rspeed)/2);
    double angle_rate_d = kp_theta*(pitch_d-(kpitch-0.447));
    angle_acc_d = angle_rate_controller.filter(angle_rate_d - (g.gyro.y - bias.y)); //change value of the shared angle_acc_d inner loop 
    /* Control System for turning */
    double yaw = PI*(Rposition-Lposition)*Dw/(SPR*Dr);

    double yaw_rate_d = kp_yaw*(goal - yaw);
    if(yaw_rate_sat < fabs(yaw_rate_d)){
      if(yaw_rate_d>0){
        yaw_rate_d = yaw_rate_sat;
      }else{
        yaw_rate_d = -yaw_rate_sat;
      }
    }
    acc_yaw_d = yaw_controller.filter(yaw_rate_d - Dw*(Rspeed-Lspeed)/2);

    unsigned long stop_time = micros();
    if (count %25 == 0){Serial.print("goal:");Serial.print(goal);Serial.print(" position:");Serial.print(position);Serial.print(" position_diff:");Serial.print(Lposition-Rposition);Serial.print(" yaw:");Serial.print(yaw);Serial.print(" acc_yaw_d:");Serial.print(acc_yaw_d);Serial.print(" angle_acc_d:");Serial.println(angle_acc_d);}
    //if (count %25 == 0){Serial.print("pos_d:");Serial.print(pos_d);Serial.print(" velocity_d:");Serial.print(velocity_d);Serial.print(" pitch_d:");Serial.print(pitch_d);Serial.print(" position:");Serial.print(position);Serial.print(" kpitch:");Serial.print(57.3*(kpitch-0.447));Serial.print(" acc:");Serial.print(angle_acc_d);Serial.print(" speed:");Serial.println((Lspeed+Rspeed)/2);}
    if (count % 8000 == 0) pos_d += 0.1;
    if (((count - 4000) % 8000 == 0) && count > 4000) goal += PI/2;
    //if (count % 4000 == 0 ) goal *= -1;
    //Serial.println(stop_time - start_time);
  } 
}


void core1( void * pvParameters ){
  int Lnext = 1, Rnext = 1;
  double Lt = 1000 , Rt = 1000;
  double Lw , Ld , Rw , Rd;
  double Lstep = 2*(PI/SPR), Rstep = 2*(PI/SPR);
  bool Ldirection = 1 , Rdirection = 1;
  digitalWrite(RdirPin, Rdirection);
  digitalWrite(LdirPin, Ldirection);
  double Racc_d,Lacc_d;
  bool R = 0, L = 0; 
  for(;;){
    unsigned long Lstart_time = micros(), Rstart_time = micros();
    double Lw0 = Lstep/Lt, Rw0 = Rstep/Rt;
    Lspeed = (Lstep > 0 ? 1 : -1)*Lw0;
    Rspeed = (Rstep > 0 ? 1 : -1)*Rw0;
    while(1){
      Lacc_d = - angle_acc_d + acc_yaw_d; // read shared value 
      Racc_d = angle_acc_d + acc_yaw_d;
      if(Lw0*Lw0 + 2*Lacc_d*Lstep < 0){
        Lstep = -Lstep;
        Ldirection = !Ldirection;
        digitalWrite(LdirPin, Ldirection);
      }
      if(Rw0*Rw0 + 2*Racc_d*Rstep < 0){
        Rstep = -Rstep;
        Rdirection = !Rdirection;
        digitalWrite(RdirPin, Rdirection);
      }
      Lw =  sqrt(Lw0*Lw0 + 2*Lacc_d*Lstep);
      Rw =  sqrt(Rw0*Rw0 + 2*Racc_d*Rstep);
      if(abs(Lacc_d)>0.0000001) Lt = (Lw - Lw0)/Lacc_d;
      if(abs(Racc_d)>0.0000001) Rt = (Rw - Rw0)/Racc_d;
      Ld = (Lstep > 0 ? 1 : -1)*Lt*1000000;
      Rd = (Rstep > 0 ? 1 : -1)*Rt*1000000;
      unsigned long current_time = micros();
      if((current_time - Lstart_time) >= Ld) L = 1;
      if((current_time - Rstart_time) >= Rd) R = 1;
      if(L&&R){
        digitalWrite(LstepPin, Lnext);
        digitalWrite(RstepPin, Rnext);
        Rposition += (Rstep > 0 ? 1 : -1);
        Lposition += (Lstep > 0 ? -1 : 1);
        Rnext = !Rnext;
        Lnext = !Lnext; 
        Lstart_time = micros();
        Rstart_time = Lstart_time;
        Lw0 = Lstep/Lt;
        Rw0 = Rstep/Rt;
        Lspeed = (Lstep > 0 ? -1 : 1)*Lw0;
        Rspeed = (Rstep > 0 ? 1 : -1)*Rw0;
        L = 0;
        R = 0;
      }
      else if(L){
        digitalWrite(LstepPin, Lnext);
        Lposition += (Lstep > 0 ? -1 : 1);
        Lnext = !Lnext;
        Lstart_time = micros();
        Lw0 = Lstep/Lt;
        Lspeed = (Lstep > 0 ? -1 : 1)*Lw0;
        L = 0;
      }
      else if(R){
        digitalWrite(RstepPin, Rnext);
        Rposition += (Rstep > 0 ? 1 : -1);
        Rnext = !Rnext;
        Rstart_time = micros();
        Rw0 = Rstep/Rt;
        Rspeed = (Rstep > 0 ? 1 : -1)*Rw0;
        R = 0;
      }
      if(rst){
        rst = 0;
        Rw0 = 0; Lw0 = 0;
        Lnext = 1; Rnext = 1;
        Lt = 1000 ; Rt = 1000;
        Lstep = 2*(PI/SPR), Rstep = 2*(PI/SPR);
        Ldirection = 1 ; Rdirection = 1;
        Lposition = 0; Rposition = 0;
      }
    }
  }
}

void loop() {
  
}