#include <arduino.h>
#include "battery.h"
#include "vehicleConfig.h"
#include "rc_board.h"
#include "radio.h"
#include "motors.h"
#include "lights.h"


bool battSense;

void checkBattery() 
{ 
  vehicleConfig* cfg = getConfigptr();
  RcData* data = getDataptr();

  if (cfg->getboardVersion() < 1.2) battSense = false;
  else battSense = true;

  // switch between load and no load contition
  if (millis() - getMillisLightsOff() >= 1000) { // one s after the vehicle did stop
    setIsDriving(false);// = false; // no load
  }
  else {
    setIsDriving(true);// = true; // under load
  }

  // Every 1000 ms, take measurements
  static unsigned long lastTrigger;
  if (millis() - lastTrigger >= 1000) {
    lastTrigger = millis();

    // Read both averaged voltages
    data->setVoltage(batteryAverage());
    //payload->vcc = vccAverage();

  }
}

// Voltage read & averaging subfunctions -----------------------------------------
// vcc ----
float vccAverage()
{
  static int raw[6];

  if (raw[0] == 0) {
    for (int i = 0; i <= 5; i++) {
      raw[i] = readVcc(); // Init array
    }
  }

  raw[5] = raw[4];
  raw[4] = raw[3];
  raw[3] = raw[2];
  raw[2] = raw[1];
  raw[1] = raw[0];
  raw[0] = readVcc();
  float average = (raw[0] + raw[1] + raw[2] + raw[3] + raw[4] + raw[5]) / 6000.0;
  return average;
}

// battery ----
float batteryAverage()
{

  vehicleConfig* cfg = getConfigptr();
  static int raw[6];

  if (!battSense) return 0;

  if (raw[0] == 0) {
    for (int i = 0; i <= 5; i++) {
      raw[i] = analogRead(PIN_ADC_VBAT); // Init array
    }
  }

  raw[5] = raw[4];
  raw[4] = raw[3];
  raw[3] = raw[2];
  raw[2] = raw[1];
  raw[1] = raw[0];
  if (getIsDriving() && cfg->getHP()) raw[0] = (analogRead(PIN_ADC_VBAT) + 31); // add 0.3V while driving (HP version only): 1023 steps * 0.3V / 9.9V = 31
  else raw[0] = analogRead(PIN_ADC_VBAT); // else take the real voltage (compensates voltage drop while driving)
  float average = (raw[0] + raw[1] + raw[2] + raw[3] + raw[4] + raw[5]) / 619.999; // 1023steps / 9.9V * 6 = 619.999
  return average;
}