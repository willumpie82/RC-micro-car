/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include <ticker.h>
#include "hall.h"
#include "rc_board.h"
#include <math.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
//#include "Wire.h"
#include "MC34933.h"


bool ms_task_flag = false;
bool decims_task_flag = false;
bool second_task_flag = false;

//function prototypes
void Tick_handle_msTask (void)
{
  ms_task_flag = true;
}
void Tick_handle_10msTask( void )
{
  decims_task_flag = true;
}
void Tick_handle_1sTask( void )
{
  second_task_flag = true;
}


Ticker ms_ticker(Tick_handle_msTask, 1, 0, MILLIS);
Ticker decims_ticker(Tick_handle_10msTask, 10, 0, MILLIS);
Ticker second_ticker(Tick_handle_1sTask, 1000,0, MILLIS);



MPU6050 mpu;
MC34933 motor1(PIN_M1A,PIN_M1B);
MC34933 steering(PIN_M2A, PIN_M2B);

bool blinkState = false;
// Variables will change:
int ledState = LOW;             // ledState used to set the LED
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 1000;           // interval at which to blink (milliseconds)
const long main_loop_interval = 10;
const long second_interval = 100;     //number of decims-ticks
unsigned long decims_counter = 0;

#define MOTORSPEED  1


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void handle_msTask (void)
{
ms_task_flag = true;
}

void handle_10msTask( void )
{
  motor1.run();
}

void handle_1sTask (void )
{
 if (ledState == LOW) 
    {
      ledState = HIGH;
      motor1.forward(MOTORSPEED);
    } 
    else 
    {
      ledState = LOW;
      motor1.backward(MOTORSPEED);
    }
    Serial.print(".");
    digitalWrite(PIN_SPARE2, ledState);  
}

void init_mpu( void )
{
// //MPU6050 init
//   Serial.println("Initializing I2C devices...");
//   mpu.initialize();

//   Serial.println(F("Initializing DMP..."));
//     devStatus = mpu.dmpInitialize();

//     // supply your own gyro offsets here, scaled for min sensitivity
//     mpu.setXGyroOffset(220);
//     mpu.setYGyroOffset(76);
//     mpu.setZGyroOffset(-85);
//     mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

//     // make sure it worked (returns 0 if so)
//     if (devStatus == 0) {
//         // turn on the DMP, now that it's ready
//         Serial.println(F("Enabling DMP..."));
//         mpu.setDMPEnabled(true);

//         // enable Arduino interrupt detection
//         Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));

//         attachInterrupt(0, dmpDataReady, RISING);

//         mpuIntStatus = mpu.getIntStatus();

//         // set our DMP Ready flag so the main loop() function knows it's okay to use it
//         Serial.println(F("DMP ready! Waiting for first interrupt..."));
//         dmpReady = true;

//         // get expected DMP packet size for later comparison
//         packetSize = mpu.dmpGetFIFOPacketSize();
//     } else {
//         // ERROR!
//         // 1 = initial memory load failed
//         // 2 = DMP configuration updates failed
//         // (if it's going to break, usually the code will be 1)
//         Serial.print(F("DMP Initialization failed (code "));
//         Serial.print(devStatus);
//         Serial.println(F(")"));
//     }
}


void setup()
{
  hall_init();

  motor1.begin();
  delay(5000);





  ms_ticker.start();
  decims_ticker.start();
  second_ticker.start();
}


void loop()
{

  ms_ticker.update();
  decims_ticker.update();
  second_ticker.update();

  if( ms_task_flag)
  {
    ms_task_flag = false;
    handle_msTask();
  }

  if( decims_task_flag)
  {
    decims_task_flag = false;
    handle_10msTask();
  }

  if( second_task_flag)
  {
    second_task_flag = false;
    handle_1sTask();
  }

}
