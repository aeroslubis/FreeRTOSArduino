#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
#include <RTClib.h>

// Semaphore handler
//SemaphoreHandle_t xI2CSemaphore;
SemaphoreHandle_t interruptSemaphore;

// Queue handler
QueueHandle_t xAnalogArray;

// Task handler
TaskHandle_t TaskAnalogReadA0_Handler;
TaskHandle_t TaskAnalogReadA1_Handler;
TaskHandle_t TaskAnalogReadA2_Handler;

// Define tasks
void TaskDisplayLCD(void *pvParameters);
void TaskBlink(void *pvParameters);
void TaskSerial(void *pvParameters);
void TaskAnalogReadA0(void *pvParameters);
void TaskAnalogReadA1(void *pvParameters);
void TaskAnalogReadA2(void *pvParameters);

LiquidCrystal_PCF8574 lcd(0x27);
RTC_DS3231 rtc;
int AnalogArray[6] = {0, 0, 0, 0, 0, 0};

// The setup function run once after reset button pressed
void setup() {
    // Initialize serial comunication
    Serial.begin(9600);

    // Initialize lcd display
    Wire.begin();
    lcd.begin(16, 2);
    lcd.setBacklight(255);

    rtc.begin();

    if (rtc.lostPower()) {
        Serial.println(F("RTC lost power!"));
        digitalWrite(LED_BUILTIN, HIGH);
        delay(5000);
        digitalWrite(LED_BUILTIN, LOW);
        // When time needs to be set on a new device, or after a power loss, the
        // following line sets the RTC to the date & time this sketch was compiled
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }


/*
    // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
    // because it is sharing a resource, such as the Serial port, I2C, SPI.
    // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
    if (xI2CSemaphore == NULL) { // Check to confirm that the Serial Semaphore has not already been created.
        xI2CSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the I2C port
        if ((xI2CSemaphore) != NULL) {
            xSemaphoreGive(xI2CSemaphore);  // Make the Serial Port available for use, by "Giving" the Semaphore.
        }
    }
*/

    interruptSemaphore = xSemaphoreCreateBinary();
    if (interruptSemaphore != NULL) {
        // Attach interrupt for Arduino digital pin
        attachInterrupt(digitalPinToInterrupt(2), interruptHandler, CHANGE);
    }

    // Create a queue
    xAnalogArray = xQueueCreate(10, // Queue lengt
                                sizeof(int)); // Queue item size

    // Setup tasks to run independently
    xTaskCreate(
        TaskDisplayLCD
        ,  "L"   // A name just for humans
        ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  NULL //Parameters passed to the task function
        ,  2  // Priority, with 2 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  NULL);//Task handle

    xTaskCreate(
        TaskSerial
        ,  "S"
        ,  128  // Stack size
        ,  NULL //Parameters passed to the task function
        ,  2  // Priority
        ,  NULL);  //Task handle

    xTaskCreate(
        TaskBlink
        ,  "B"
        ,  64  // Stack size
        ,  NULL //Parameters passed to the task function
        ,  0  // Priority
        ,  NULL);  //Task handle

    xTaskCreate(
        TaskAnalogReadA0
        ,  "A0"
        ,  64  // Stack size
        ,  NULL //Parameters passed to the task function
        ,  1  // Priority
        ,  &TaskAnalogReadA0_Handler);  //Task handle

    xTaskCreate(
        TaskAnalogReadA1
        ,  "A1"
        ,  64  // Stack size
        ,  NULL //Parameters passed to the task function
        ,  1  // Priority
        ,  &TaskAnalogReadA1_Handler);  //Task handle

    xTaskCreate(
        TaskAnalogReadA2
        ,  "A2"
        ,  64  // Stack size
        ,  NULL //Parameters passed to the task function
        ,  1  // Priority
        ,  &TaskAnalogReadA2_Handler);  //Task handle
}

void loop() {
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskSerial(void* pvParameters) {
/*
 Serial
 Send "s" or "r" through the serial port to control the suspend and resume of the LED light task.
 This example code is in the public domain.
*/
    (void) pvParameters;
    //int analogValueFromQueue;
    //char serialLine0[37];

    for (;;) { // A Task shall never return or exit.
        while(Serial.available()>0) {
            switch(Serial.read()) {
                case 'a':
                    vTaskSuspend(TaskAnalogReadA0_Handler); 
                    Serial.println(F("A0 Suspended!"));
                    break;
                case 'A':
                    vTaskResume(TaskAnalogReadA0_Handler);
                    Serial.println(F("A0 Resumed!"));
                    break;
                case 'b':
                    vTaskSuspend(TaskAnalogReadA1_Handler);
                    Serial.println(F("A1 Suspended!"));
                    break;
                case 'B':
                    vTaskResume(TaskAnalogReadA1_Handler);
                    Serial.println(F("A1 Resumed!"));
                    break;
                case 'c':
                    vTaskSuspend(TaskAnalogReadA2_Handler);
                    Serial.println(F("A2 Suspended!"));
                    break;
                case 'C':
                    vTaskResume(TaskAnalogReadA2_Handler);
                    Serial.println(F("A2 Resumed!"));
                    break;
            }
            vTaskDelay(1);
        }
/*
        if (xQueueReceive(xAnalogValueSerial, &analogValueFromQueue, portMAX_DELAY) == pdPASS) {
            DateTime now = rtc.now();
            sprintf(serialLine0, "ANALOG: %-7d TIME: %02d:%02d:%02d",
                    analogValueFromQueue, now.hour(), now.minute(), now.second());
            Serial.print(serialLine0);
            Serial.print("\r");
        }
*/
        vTaskDelay(100 / portTICK_PERIOD_MS);  // one tick delay (15ms) in between reads for stability
    }
}

void TaskDisplayLCD(void *pvParameters) {
    (void) pvParameters;
    char lcdLine0[17]; // 16 character lcd
    char lcdLine1[17]; // 16 character lcd
    lcd.noBlink();

    for (;;) {
        if (xQueueReceive(xAnalogArray, &AnalogArray, portMAX_DELAY) == pdPASS) {
            sprintf(lcdLine0, "A:%02d B:%02d C:%02d", AnalogArray[1], AnalogArray[3], AnalogArray[5]);
            lcd.setCursor(0, 0); // Dont use lcd.clear() function
            lcd.print(lcdLine0);
        }
        //while (xTaskGetTickCount() - xPreviousTick < pdMS_TO_TICKS(1000)) {
         //   xPreviousTick = xTaskGetTickCount();
        DateTime now = rtc.now();
        sprintf(lcdLine1, "TIME  : %02d:%02d:%02d", now.hour(), now.minute(), now.second());
        lcd.setCursor(0, 1);
        lcd.print(lcdLine1);
        //}
        vTaskDelay(200 / portTICK_PERIOD_MS);  // one tick delay (15ms) in between reads for stability
    }
}

void TaskBlink(void *pvParameters) {  // This is a task.
    (void) pvParameters;
    pinMode(LED_BUILTIN, OUTPUT);

    for (;;) { // A Task shall never return or exit.
/*
         digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
         vTaskDelay( 30 / portTICK_PERIOD_MS ); // wait for one second
         digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
         vTaskDelay( 500 / portTICK_PERIOD_MS ); // wait for one second
*/
        if (xSemaphoreTake(interruptSemaphore, portTICK_PERIOD_MS) == pdPASS) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }
        //vTaskDelay(10);
    }
}

void TaskAnalogReadA0(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        AnalogArray[0] = 0;
        AnalogArray[1] = map(analogRead(A0), 0, 1024, 0, 99);
        // Post an item on a queue.
        xQueueSend(xAnalogArray, &AnalogArray, portMAX_DELAY);
        vTaskDelay(100 / portTICK_PERIOD_MS);  // one tick delay (15ms) in between reads for stability
    }
}

void TaskAnalogReadA1(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        AnalogArray[2] = 0;
        AnalogArray[3] = map(analogRead(A1), 0, 1024, 0, 99);
        // Post an item on a queue.
        xQueueSend(xAnalogArray, &AnalogArray, portMAX_DELAY);
        vTaskDelay(100 / portTICK_PERIOD_MS);  // one tick delay (15ms) in between reads for stability
    }
}

void TaskAnalogReadA2(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        AnalogArray[4] = 0;
        AnalogArray[5] = map(analogRead(A2), 0, 1024, 0, 99);
        // Post an item on a queue.
        xQueueSend(xAnalogArray, &AnalogArray, portMAX_DELAY);
        vTaskDelay(100 / portTICK_PERIOD_MS);  // one tick delay (15ms) in between reads for stability
    }
}

void interruptHandler() {
    xSemaphoreGiveFromISR(interruptSemaphore, NULL);
}
