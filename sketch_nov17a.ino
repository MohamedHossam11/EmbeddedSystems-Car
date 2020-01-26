
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

#include <LiquidCrystal.h>
#include <Arduino_FreeRTOS.h>
#include <event_groups.h>
#include <FreeRTOSConfig.h>
#include <semphr.h>
#include <queue.h>
#include <list.h>
#include <timers.h>
#include <portable.h>
#include <projdefs.h>
#include <task.h>
#include <croutine.h>
#include <portmacro.h>
#include <mpu_wrappers.h>
#include <FreeRTOSVariant.h>
#include <dht11.h>

#include <Wire.h>
#include "RTClib.h"
RTC_DS3231 rtc;

# define Start_Byte 0x7E
# define Version_Byte 0xFF
# define Command_Length 0x06
# define End_Byte 0xEF
# define Acknowledge 0x00 //Returns info with command 0x41 [0x01: info, 0x00: no info]

SoftwareSerial mySoftwareSerial(51, 53); // RX, TX
DFRobotDFPlayerMini myDFPlayer;



const int SW_pin = 2;
int buzzer = 43;
int X_pin = A0;
int Y_pin = A1;
int in1 = 31;
int in2 = 33;
int in3 = 35;
int in4 = 37;
int echo = 39;
int trigger = 41;
int light = A2;
int LED1 = 45;
int LED2 = 47;
int temprature = A8;
int buttonPlay = 23;
int buttonNext = 22;
int buttonPrev = 24;
bool played = false;
bool paused = false;
int song = 1;

char gear = 'N';

dht11 DHT11;

SemaphoreHandle_t xBinarySemaphore;

LiquidCrystal lcd(30, 32, 9, 10, 11, 12);


int PWM3 = 3;
int PWM4 = 4;

long duration;
long durationSecondSensor;
int distance;
int distanceSecondSensor;

void TaskMove( void *pvParameters );
void TaskModule2( void *pvParameters );
void TaskModule3( void *pvParameters );




void setup() {
  Serial.begin(9600);

  mySoftwareSerial.begin(9600);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
  }
  Serial.println(F("DFPlayer Mini online."));

  Wire.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  xBinarySemaphore = xSemaphoreCreateBinary();


  pinMode(SW_pin, INPUT);

  lcd.begin(16, 2);


  pinMode(light, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(buzzer, OUTPUT);

  pinMode(in2, OUTPUT);
  pinMode(in1, OUTPUT);

  pinMode(in4, OUTPUT);
  pinMode(in3, OUTPUT);

  pinMode(PWM3, OUTPUT);
  pinMode(PWM4, OUTPUT);

  pinMode(echo, INPUT);
  pinMode(trigger, OUTPUT);
  pinMode(buttonPlay, INPUT_PULLUP);
  pinMode(buttonNext, INPUT_PULLUP);
  pinMode(buttonPrev, INPUT_PULLUP);


  xTaskCreate(TaskMove, "Move", 1000, NULL, 2, NULL);
  xTaskCreate(TaskModule2, "Module2", 1000, NULL, 1, NULL);
  xTaskCreate(TaskModule3, "Module2", 1000, NULL, 1, NULL);
}

void loop() {

}


void TaskMove(void *pvParameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    digitalWrite(trigger, LOW);
    delayMicroseconds(5);
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);
    duration = pulseIn(echo, HIGH);
    distance = duration * 0.034 / 2;
    delay(50);




    if (distance > 15 && analogRead(Y_pin) < 470) {

      analogWrite(PWM3, 70);
      analogWrite(PWM4, 70);

      digitalWrite(in1, HIGH);

      digitalWrite(in2, LOW);

      digitalWrite(in3, LOW);

      digitalWrite(in4, HIGH);

      digitalWrite(buzzer, LOW);

      gear = 'D';

    }
    else if (analogRead(Y_pin) > 554 ) {
      analogWrite(PWM3, 70);
      analogWrite(PWM4, 70);

      digitalWrite(in1, LOW);

      digitalWrite(in2, HIGH);

      digitalWrite(in3, HIGH);

      digitalWrite(in4, LOW);

      digitalWrite(buzzer, LOW);

      gear = 'R';

    }
    else if (distance < 15)
    {

      digitalWrite(in1, LOW);

      digitalWrite(in2, LOW);

      digitalWrite(in3, LOW);

      digitalWrite(in4, LOW);

      digitalWrite(buzzer, HIGH);

      gear = 'N';
    }
    else
    {

      digitalWrite(in1, LOW);

      digitalWrite(in2, LOW);

      digitalWrite(in3, LOW);

      digitalWrite(in4, LOW);

      digitalWrite(buzzer, LOW);

      gear = 'N';


    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( 250 ) );

  }
}

void TaskModule2(void *pvParameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    DateTime now = rtc.now();
    delay(1000);
    lcd.setCursor(0, 0);
    lcd.print((int)(analogRead(temprature) * 0.48828125));
    lcd.setCursor(2, 0);
    lcd.print("C");
    lcd.setCursor(15, 0);
    lcd.print(gear);
    lcd.setCursor(0, 1);
    lcd.print(now.year());
    lcd.setCursor(4, 1);
    lcd.print("/");
    lcd.setCursor(5, 1);
    lcd.print(now.month());
    lcd.setCursor(7, 1);
    lcd.print("/");
    lcd.setCursor(8, 1);
    lcd.print(now.day());
    lcd.setCursor(10, 1);
    lcd.print(" ");
    lcd.setCursor(11, 1);
    lcd.print(now.hour());
    lcd.setCursor(13, 1);
    lcd.print(":");
    lcd.setCursor(14, 1);
    lcd.print(now.minute());

    if (analogRead(light) > 600) {
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
    }
    else {
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
    }

    //if(digitalRead(SW_pin)== HIGH){
    //digitalWrite(buzzer,HIGH);
    //digitalWrite(LED1,HIGH);
    //digitalWrite(LED1,HIGH);}

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( 250 ) );
  }
}

void TaskModule3(void *pvParameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();


  while (1) {
    Serial.print(digitalRead(buttonPlay));

    myDFPlayer.volume(20);
    if ((digitalRead(buttonPlay)) == LOW && !played) {
      myDFPlayer.start();
      played = true;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

      xSemaphoreGive( xBinarySemaphore);
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( 1000 ) );

    }
    else if ((digitalRead(buttonPlay)) == LOW && played) {
      myDFPlayer.pause();
      Serial.print("pause");
      played = false;
      xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );

      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( 1000 ) );

    }
    else if ((digitalRead(buttonPrev)) == LOW && xSemaphoreTake( xBinarySemaphore, portMAX_DELAY )) {
      myDFPlayer.previous();

      played = true;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

      xSemaphoreGive( xBinarySemaphore);

      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( 1000 ) );

    }
    else if ((digitalRead(buttonNext)) == LOW && xSemaphoreTake( xBinarySemaphore, portMAX_DELAY )) {
      myDFPlayer.next();
      played = true;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

      xSemaphoreGive( xBinarySemaphore);

      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( 1000 ) );

    } else {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( 250 ) );
    }

  }
}
