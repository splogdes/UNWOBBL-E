#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <cmath>
#include "Classes.h"

#define SPR 6400.0
#define RdirPin 4
#define RstepPin 16
#define LdirPin 15
#define LstepPin 2
#define Dw 0.0816
#define Dr 0.13
#define BUFFER_SIZE 32

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

ESP32SPISlave slave;

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

void add_transaction(String data){
    uint8_t* rx_buf = (uint8_t*)calloc(BUFFER_SIZE, 8);
    uint8_t* tx_buf = (uint8_t*)calloc(BUFFER_SIZE, 8);

    memset(rx_buf, 0, BUFFER_SIZE);
    memset(tx_buf, 0, BUFFER_SIZE);

    for (int i = 0; i<data.length(); i++){
        tx_buf[i] = data[i];
    }

    slave.queue(rx_buf, tx_buf, BUFFER_SIZE*2);
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

  PID angle_rate_controller(60,5,0.01);
  double kp_theta = 41;
  double angle_sat = 0.02; 
  PID velocity_controller(-0.003,-0.0007,-0.00001,angle_sat);
  double kp_pos = 5;
  double sat_velocity = 1;
  double kp_yaw = 2;
  double yaw_rate_sat = 0.2;
  PID yaw_controller(120,20,0.005);
  Kalman_filter_pitch pitch_filter(0.001,0.003,0.03);

  dir bias;

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  /* corey's bit */
  slave.setDataMode(SPI_MODE0);
  slave.begin(VSPI);

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);

  bias = bias_gyro(&mpu);

  int count = 0;
  int flag = 0;
  int instruction = 0;
  double akp,aki,akd,vkp,vki,vkd;
  double pos_d = 0;
  double goal = 0;

  /* XY position */
  double last_pos = 0, current_pos;
  double X = 0 ,Y = 0;
  double current_yaw;
  unsigned long current_time = micros();
  double offset = 0.035;
  double position = 0;
  double kpitch = 0;
  double yaw = 0;

  angle_rate_controller.init();
  velocity_controller.init();
  pitch_filter.init();
  yaw_controller.init();

  for(;;){

    if(flag == 0b11){
      switch(instruction){
        case 0: pos_d = 1; break;
        case 1: goal = -1.9; break;
        case 2: pos_d = 2; break;
        case 3: pos_d = 1; break;
        case 4: goal = 0; break;
        case 5: pos_d = 0; break;
        case 6: goal = 2*PI; break;
        case 7: instruction = -1; break;
      }
      instruction++;
    }

    /* Work out position */
    current_pos = position;
    current_yaw = yaw;
    X += (current_pos-last_pos)*cos(current_yaw);
    Y += (current_pos-last_pos)*sin(current_yaw);
    last_pos = current_pos;

    // if(Serial.available()){
    //   Serial.println("");
    //   angle_sat = Serial.parseFloat();
    //   sat_velocity = Serial.parseFloat();
    //   kp_pos = Serial.parseFloat();
    //   vkp = Serial.parseFloat();
    //   vki = Serial.parseFloat();
    //   vkd = Serial.parseFloat();
    //   velocity_controller.set_coefficients(vkp,vki,vkd,angle_sat);
    //   angle_acc_d = 0;
    //   Lposition = 0;
    //   Rposition = 0;
    //   velocity_controller.init();
    //   angle_rate_controller.init();
    //   yaw_controller.init();
    //   rst = 1;
    //   count = 0;
    //   goal = 0;
    //   pos_d = 0;
    // }
    count++;
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    double pitch = atan2(a.acceleration.z,sqrt(a.acceleration.x*a.acceleration.x+a.acceleration.y*a.acceleration.y));
    kpitch = pitch_filter.filter(g.gyro.y - bias.y,pitch);
    /* Control System for balance */
    position = Dw*PI*double(Lposition+Rposition)/(2*SPR);
    double velocity_d = kp_pos*(pos_d - position);
    if(sat_velocity < fabs(velocity_d)){
      if(velocity_d>0){
        velocity_d = sat_velocity;
      }else{
        velocity_d = -sat_velocity;
      }
    }
    double pitch_d = velocity_controller.filter(velocity_d - (Lspeed+Rspeed)/2);
    double angle_rate_d = kp_theta*(pitch_d-(kpitch-offset));
    angle_acc_d = angle_rate_controller.filter(angle_rate_d - (g.gyro.y - bias.y)); //change value of the shared angle_acc_d inner loop 
    /* Control System for turning */
    yaw = PI*(Rposition-Lposition)*Dw/(SPR*Dr);

    double yaw_rate_d = kp_yaw*(goal - yaw);
    if(yaw_rate_sat < fabs(yaw_rate_d)){
      if(yaw_rate_d>0){
        yaw_rate_d = yaw_rate_sat;
      }else{
        yaw_rate_d = -yaw_rate_sat;
      }
    }
    acc_yaw_d = yaw_controller.filter(yaw_rate_d - Dw*(Rspeed-Lspeed)/2);

    //if (count %25 == 0){Serial.print("goal:");Serial.print(goal);Serial.print(" position:");Serial.print(position);Serial.print(" position_diff:");Serial.print(Lposition-Rposition);Serial.print(" yaw:");Serial.print(yaw);Serial.print(" acc_yaw_d:");Serial.print(acc_yaw_d);Serial.print(" angle_acc_d:");Serial.print(angle_acc_d);Serial.print(" flag:");Serial.println(flag);}
    if (count %25 == 0){Serial.print("pos_d:");Serial.print(pos_d);Serial.print(" velocity_d:");Serial.print(velocity_d);Serial.print(" pitch_d:");Serial.print(pitch_d);Serial.print(" position:");Serial.print(position);Serial.print(" kpitch:");Serial.print(1000*(kpitch));Serial.print(" offset:");Serial.print(573*offset);Serial.print(" acc:");Serial.print(angle_acc_d);Serial.print(" speed:");Serial.println((Lspeed+Rspeed)/2);}

    /* Test Stuff */
    if (count == 1000){
      Lposition = 0;
      Rposition = 0;
    }
    //if (count % 8000 == 0) goal = 0;
    // if ((count-8000) % 16000 == 0 && count >= 8000) pos_d += 0.45;
    // if (count % 16000 == 0 ) goal += PI/2;
    // if ((count-8000) % 16000 == 0 && count >= 8000) pos_d += 1;
    // if ((count-8000) % 16000 == 0 && count >= 8000) pos_d -= 1;
    // if (count % 16000 == 0 ) goal -= PI/2;
    // if ((count-8000) % 16000 == 0 && count >= 8000) pos_d -= 0.45;

    /* Bit of maze */
    // int current = count % 50000;
    // if(current < 2000) pos_d = 0;
    // else if(current < 10000) pos_d = 0.45;
    // else if(current < 14000) goal = PI/2;
    // else if(current < 26000) pos_d = 1.45;
    // else if(current < 38000) pos_d = 0.45; 
    // else if(current < 42000) goal = 0;
    // else if(current < 50000) pos_d = 0; 
    //Serial.println(stop_time - start_time);

    if(fabs(position-pos_d) < 0.03 ){
      if(micros() - current_time > 3000000){
        flag |= 0b10;
      }
    }
    else{
      flag &= 0b01;
      current_time = micros();
    }

    if(fabs(yaw-goal) < 0.05 ){
      if(micros() - current_time > 1000000){
        flag |= 0b01;
      }
    }
    else{
      flag &= 0b10;
      current_time = micros();
    }


    char buffer[100];
    if (count %40 == 0){
      snprintf(buffer, 100, "{\"X\":%f,\"Y\":%f,\"Yaw\":%f,\"Flag\":%d}",float(X),float(Y),float(yaw),flag);
      add_transaction((String)buffer);
      //Serial.println(slave.getNewestMessage());
    }
 
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
    Lspeed = (Lstep > 0 ? -1 : 1)*Lw0;
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
      // if(L&&R){
      //   digitalWrite(LstepPin, Lnext);
      //   digitalWrite(RstepPin, Rnext);
      //   Rposition += (Rstep > 0 ? 1 : -1);
      //   Lposition += (Lstep > 0 ? -1 : 1);
      //   Rnext = !Rnext;
      //   Lnext = !Lnext; 
      //   Lstart_time = micros();
      //   Rstart_time = Lstart_time;
      //   Lw0 = Lstep/Lt;
      //   Rw0 = Rstep/Rt;
      //   Lspeed = (Lstep > 0 ? -1 : 1)*Lw0;
      //   Rspeed = (Rstep > 0 ? 1 : -1)*Rw0;
      //   L = 0;
      //   R = 0;
      // }
      if(L){
        digitalWrite(LstepPin, Lnext);
        Lposition += (Lstep > 0 ? -1 : 1);
        Lnext = !Lnext;
        Lstart_time = micros();
        Lw0 = Lstep/Lt;
        Lspeed = (Lstep > 0 ? -1 : 1)*Lw0;
        L = 0;
      }
      if(R){
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