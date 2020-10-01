#include <Arduino.h>
#include "lights.h"
#include "radio.h"
#include "vehicleConfig.h"
#include "motors.h"
#include "rc_board.h"

#include <statusLED.h> // https://github.com/TheDIYGuy999/statusLED


// Headlight off delay
unsigned long millisLightOff = 0;


// Status LED objects
statusLED tailLight(false); // "false" = output not inverted
statusLED headLight(false);
statusLED indicatorL(false);
statusLED indicatorR(false);
statusLED beaconLights(false);

// Indicators
bool left;
bool right;
bool hazard;

bool getLeft( void )
{
    return left;
}

bool getRight( void )
{
    return right;
}

bool getHazard( void )
{
    return hazard;
}

void SetmillisLightOff( unsigned long millis)
{
    millisLightOff = millis;
}

void UpdatemillisLightOff( void )
{
    millisLightOff = millis();
}


unsigned long getMillisLightsOff( void )
{
  return millisLightOff;
}

void setupLights( void )
{
#ifdef CONFIG_HAS_LIGHTS
  // LED setup
  if (vehicleType == 4 || vehicleType == 5 ) indicators = false; // Indicators use the same pins as the MPU-6050, so they can't be used in vehicleType 4 or 5!

  if (tailLights) tailLight.begin(A1); // A1 = Servo 2 Pin
  if (headLights) headLight.begin(0); // 0 = RXI Pin
  if (indicators) {
    indicatorL.begin(A4); // A4 = SDA Pin
    indicatorR.begin(A5); // A5 = SCL Pin
  }
  if (beacons) beaconLights.begin(A3); // A3 = Servo 4 Pin
#endif //CONFIG_HAS_LIGHTS
}

//
// =======================================================================================================
// LED
// =======================================================================================================
//

// Brake light subfunction for ESC vehicles
bool escBrakeActive() {
  RcData* data = getDataptr();
  static byte driveState;
  bool brake= false;

  switch (driveState) { // 0 = neutral, 1 = forward, 2 = reverse, 3 = brake

    case 0: // neutral
      if (data->axis3 > 55) driveState = 1; // forward
      if (data->axis3 < 45) driveState = 2; // reverse
      brake = false;
      break;

    case 1: // forward
      if (data->axis3 < 45) driveState = 3; // brake
      brake = false;
      break;

    case 2: // reverse
      if (data->axis3 > 55) driveState = 1; // forward
      brake = false;
      break;

    case 3: // brake
      if (data->axis3 > 45) driveState = 2; // go to reverse, if above neutral
      brake = true;
      break;
      default:
      {
        brake = false;
      }

  }
  return brake;
}

void led() 
{
  vehicleConfig* cfg = getConfigptr();
  RcData* data = getDataptr();
  // Lights are switching off 10s after the vehicle did stop
  if (millis() - millisLightOff >= 10000) {
    headLight.off(); // Headlight off
    tailLight.off(); // Taillight off
    beaconLights.off(); // Beacons off
  }
  else {
    if (!cfg->getescBrakeLights() && ((!cfg->getHP() && motorBrakeActive(MOTOR1)) || (cfg->getHP() && motorBrakeActive(MOTOR2)) )) { // if braking detected from TB6612FNG motor driver
      tailLight.on(); // Brake light (full brightness)
    }

    else if (cfg->getescBrakeLights() && escBrakeActive() ) { // or braking detected from ESC
      tailLight.on(); // Brake light (full brightness)
    }

    else {
      tailLight.flash(10, 14, 0, 0); // Taillight: 10 on  / 14 off = about 40% brightness (soft PWM)
    }
    beaconLights.flash(50, 650, 0, 0); // Simulate rotating beacon lights with short flashes
    headLight.on(); // Headlight on
  }

  // Indicator lights ----
  if (cfg->getindicators()) {
    // Set and reset by lever
    if (data->axis4 < 5) left = true;
    if (data->axis4 > 55) left = false;

    if (data->axis4 > 95) right = true;
    if (data->axis4 < 45) right = false;

    // Reset by steering
    static int steeringOld;

    if (data->axis1 > steeringOld + 10) {
      left = false;
      steeringOld = data->axis1;
    }

    if (data->axis1 < steeringOld - 10) {
      right = false;
      steeringOld = data->axis1;
    }

    // Lights
    if (left) { // Left indicator
      right = false;
      indicatorL.flash(375, 375, 0, 0);
      indicatorR.off();
    }

    if (right) { // Right indicator
      left = false;
      indicatorR.flash(375, 375, 0, 0);
      indicatorL.off();
    }

    if (hazard) { // Hazard lights
      if (left) {
        left = false;
        indicatorL.off();
      }
      if (right) {
        right = false;
        indicatorR.off();
      }
      indicatorL.flash(375, 375, 0, 0);
      indicatorR.flash(375, 375, 0, 0);
    }

    if (!hazard && !left && !right) {
      indicatorL.off();
      indicatorR.off();
    }
  }
}