#include <Arduino.h>

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
volatile int share = 0;

void setup() {
  Serial.begin(115200); 
  createSemaphore();
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
}


void Task1code( void * pvParameters ){
  for(;;){
    Serial.print("Task1 running on core ");
    Serial.println(xPortGetCoreID());
    lockVariable();
    share++;
    unlockVariable();
    vTaskDelay(100);
  } 
}


void Task2code( void * pvParameters ){
  for(;;){
    Serial.print("Task2 running on core ");
    Serial.println(xPortGetCoreID());
    lockVariable();
    Serial.println(share);
    unlockVariable();
    vTaskDelay(100);
  }
}

void loop() {
  
}