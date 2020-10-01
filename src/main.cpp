// 4 Channel "Micro RC" Receiver with 4 standard RC Servo Outputs
// ATMEL Mega 328P TQFP 32 soldered directly to the board, 8MHz external resonator,
// 2.4GHz NRF24L01 SMD radio module, TB6612FNG dual dc motor driver
// An MPU-6050 gyro / accelerometer can be used for MRSC stability control or self balancing robots
// SBUS output on pin TXO

// See: https://www.youtube.com/playlist?list=PLGO5EJJClJBCjIvu8frS7LrEU3H2Yz_so

// * * * * N O T E ! The vehicle specific configurations are stored in "vehicleConfig.h" * * * *

const float codeVersion = 3.4; // Software revision (see https://github.com/TheDIYGuy999/Micro_RC_Receiver/blob/master/README.md)


// Libraries
#include <Arduino.h>
#include "vehicleConfig.h"
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

#define CLIENT_ADDRESS 1   
#define SERVER_ADDRESS 2

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

radio_c radio(CLIENT_ADDRESS,PIN_NRF_CE, PIN_NRF_CSN);



void setup() 
{
  setupConfig(STANDARDCAR);
  
  vehicleConfig* cfg = getConfigptr();

#ifdef DEBUG  
  Serial.begin(115200);
   #ifdef STM32F1xx
    USART1->BRR = 0x271;
  #endif
  //printf_begin();
  cfg->serialCommands = false;
  delay(3000);
#else
  // If TXO pin or RXI pin is used for other things, disable Serial
  if (cfg->getTXO_momentary1() || cfg->getTXO_toggle1() || cfg->getheadLights()) 
  {
    Serial.end(); // make sure, serial is off!
    #ifndef STM32F1xx
      UCSR0B = 0b00000000;
    #endif
    cfg->serialCommands = false;
  }
#endif

#ifdef CONFIG_HAS_TONE
  if(cfg->gettoneOut())
  {
    R2D2_tell();
  }
#endif
  //setup lights
  setupLights();

  // Radio setup
  radio.begin();

  // Servo pins
  setupServos();

  // Special functions
  if (cfg->getTXO_momentary1() || cfg->getTXO_toggle1()) pinMode(PIN_SPARE1, OUTPUT);

  // Motor driver setup
  setupMotors();

  if (cfg->usesMPU()) // Only for self balancing vehicles and cars with MRSC
  { 
    // setup MPU 6050 accelerometer / gyro setup
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

void digitalOutputs() 
{

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
  radio.receiveData();

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
  // Normal protocol (for ESP32 engine sound controller only)
  sendSerialCommands();
}

