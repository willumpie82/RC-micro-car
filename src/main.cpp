// 4 Channel "Micro RC" Receiver with 4 standard RC Servo Outputs
// ATMEL Mega 328P TQFP 32 soldered directly to the board, 8MHz external resonator,
// 2.4GHz NRF24L01 SMD radio module, TB6612FNG dual dc motor driver
// An MPU-6050 gyro / accelerometer can be used for MRSC stability control or self balancing robots
// SBUS output on pin TXO

// See: https://www.youtube.com/playlist?list=PLGO5EJJClJBCjIvu8frS7LrEU3H2Yz_so

// * * * * N O T E ! The vehicle specific configurations are stored in "vehicleConfig.h" * * * *

const float codeVersion = 3.4; // Software revision (see https://github.com/TheDIYGuy999/Micro_RC_Receiver/blob/master/README.md)

//
// =======================================================================================================
// BUILD OPTIONS (comment out unneeded options)
// =======================================================================================================
//

//#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!

//
// =======================================================================================================
// INCLUDE LIRBARIES
// =======================================================================================================
//

// Libraries
#include <Wire.h> // I2C library (for the MPU-6050 gyro /accelerometer)
//#include <RF24.h> // Installed via Tools > Board > Boards Manager > Type RF24
#include <Servo.h>
#include "vehicleConfig.h"
// Tabs (header files in sketch directory)
#include "readVCC.h"
#include "steeringCurves.h"
#include "tone.h"
#include "MPU6050.h"
#include "helper.h"
#include "radio.h"
#include "motors.h"
#include "battery.h"
#include "hall.h"
#include "rc_board.h"
#include "servos.h"
#include "balancing.h"
#include "lights.h"






//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void setup() 
{

  vehicleConfig* cfg = getConfigptr();
  cfg->begin(COKECANCAR);

  RcData* data = getDataptr();

#ifdef DEBUG  
  Serial.begin(115200);
  printf_begin();
  serialCommands = false;
  delay(3000);
#endif

#ifndef DEBUG
  // If TXO pin or RXI pin is used for other things, disable Serial
  if (cfg->getTXO_momentary1() || cfg->getTXO_toggle1() || cfg->getheadLights()) 
  {
    Serial.end(); // make sure, serial is off!
    #ifndef STM32F1xx
      UCSR0B = 0b00000000;
    #endif
    cfg->serialCommands = false;
  }
  else 
  { // Otherwise use it for serial commands to the light and sound controller

#ifdef CONFIG_HAS_SBUS
    SBUS* x8r = getSBUSptr();
    x8r->begin(); // begin the SBUS communication
#else
    Serial.begin(115200); // begin the standart serial communication
#endif
    cfg->serialCommands = true;
  }
#endif

  if(cfg->gettoneOut())
  {
    R2D2_tell();
  }


  // LED setup
#ifdef CONFIG_HAS_LIGHTS
  if (vehicleType == 4 || vehicleType == 5 ) indicators = false; // Indicators use the same pins as the MPU-6050, so they can't be used in vehicleType 4 or 5!

  if (tailLights) tailLight.begin(A1); // A1 = Servo 2 Pin
  if (headLights) headLight.begin(0); // 0 = RXI Pin
  if (indicators) {
    indicatorL.begin(A4); // A4 = SDA Pin
    indicatorR.begin(A5); // A5 = SCL Pin
  }
  if (beacons) beaconLights.begin(A3); // A3 = Servo 4 Pin
#endif //CONFIG_HAS_LIGHTS

// Radio setup
#ifdef CONFIG_HAS_NRF24
  setupRadio();
#endif //CONFIG_HAS_NRF24

  // Servo pins
#ifdef CONFIG_HAS_SERVO
  servo1.attach(A0);
  if (!tailLights) servo2.attach(A1);
  if (!engineSound && !toneOut) servo3.attach(A2);
  if (!beacons) servo4.attach(A3);
#endif //CONFIG_HAS_SERVO

  // All axes to neutral position
  data->axis1 = 50;
  data->axis2 = 50;
  data->axis3 = 50;
  data->axis4 = 50;
  data->pot1 = 50; // Added in v3.32

  

  // Special functions
  if (cfg->getTXO_momentary1() || cfg->getTXO_toggle1()) pinMode(PIN_SPARE1, OUTPUT);

  // Motor driver setup
  setupMotors();

  if (cfg->getvehicleType() == balancingthing || cfg->getvehicleType() == carwithMRSC) { // Only for self balancing vehicles and cars with MRSC
    // MPU 6050 accelerometer / gyro setup
    setupMpu6050();

    // PID controller setup
    setupPid();
  }
}



//
// =======================================================================================================
// WRITE DIGITAL OUTPUTS (SPECIAL FUNCTIONS)
// =======================================================================================================
//

void digitalOutputs() {

  vehicleConfig* cfg = getConfigptr();
  RcData* data = getDataptr();
  static bool wasPressed;

  if (cfg->getTXO_momentary1()) { // only, if momentary function is enabled in vehicle configuration
    if (data->momentary1) {
      digitalWrite(PIN_SPARE1, HIGH);
      #ifdef CONFIG_HAS_TONE
        R2D2_tell();
      #endif
    }
    else digitalWrite(PIN_SPARE1, LOW);
  }

  if (cfg->getTXO_toggle1()) { // only, if toggle function is enabled in vehicle configuration

    if (data->momentary1  && !wasPressed) {
      digitalWrite(PIN_SPARE1, !digitalRead(PIN_SPARE1));
      wasPressed = true;
    }
    if (!data->momentary1) wasPressed = false;
  }
}






//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//

void loop() {
  // Read radio data from transmitter
  readRadio();

  // Write the servo positions
  writeServos();

  // Drive the motors
  drive();

  // Battery check
  checkBattery();

  // Digital Outputs (special functions)
  digitalOutputs();

  // LED
  led();

  // Send serial commands
#ifdef CONFIG_HAS_SBUS
  // Serial commands are transmitted in SBUS standard
  sendSbusCommands();
#else
  // Normal protocol (for ESP32 engine sound controller only)
  sendSerialCommands();
#endif
}

