#include <Arduino.h>
#include "motors.h"
#include "vehicleConfig.h"
#include "rc_board.h"
#include "radio.h"
#include "lights.h"
#include "steeringCurves.h"
#include "balancing.h"


#include "../lib/MC34933/MC34933.h"  // https://github.com/willumpie82/MC34933
#include <../lib/PWMFrequency/PWMFrequency.h> // https://github.com/TheDIYGuy999/PWMFrequency

bool isDriving; // is the vehicle driving?
// Motor objects
MC34933 Motor1;
MC34933 Motor2;


// This array is intended for the "vhctype_semi caterpillar" mode. The inner wheel can max. slow down to 60% of the
// outer wheels RPM
float curvevhctype_semi[][2] = {  // see excel sheet!
  {0, 60} // {input value, output value}
  , {25, 70}
  , {50, 80}
  , {75, 90}
  , {100, 100}
};

// This array is intended for the "Caterpillar" mode. The inner wheel can spin backwars  up to 100% of the
// outer wheels RPM. That allows for turning the vehicle "on place"
float curveFull[][2] = {
  {0, -100} // {input value, output value}
  , {7, 0}
  , {20, 45}
  , {60, 85}
  , {100, 100}
};

// This array is intended for the "Differential Thrust" mode. The inner motor can max. slow down to 20% of the
// outer motors RPM
float curveThrust[][2] = {  // see excel sheet!
  {0, 20} // {input value, output value}
  , {25, 40}
  , {50, 60}
  , {75, 80}
  , {100, 100}
};

//
// =======================================================================================================
// MOTOR DRIVER SETUP
// =======================================================================================================
//

void setupMotors() 
{
  vehicleConfig* cfg = getConfigptr();
  // MC34933 H-Bridge pins
  const byte motor1_in1 = PIN_MOTOR_M1A;
  const byte motor1_in2 = PIN_MOTOR_M1B;

  const byte motor2_in1 = PIN_MOTOR_M2A; 
  const byte motor2_in2 = PIN_MOTOR_M2B; 
  

  // SYNTAX: IN1, IN2, PWM, min. input value, max. input value, neutral position width
  // invert rotation direction true or false
  Motor1.begin(motor1_in1, motor1_in2, 0, 100, 4, false); // Drive motor
  Motor2.begin(motor2_in1, motor2_in2, 0, 100, 4, false); // Steering motor (Drive in "HP" version)

  // Motor PWM frequency prescalers (Requires the PWMFrequency.h library)
  // Differential steering vehicles: locked to 984Hz, to make sure, that both motors use 984Hz.
  if (cfg->getvehicleType() == vhctype_semi || cfg->getvehicleType() == vhctype_unknown || cfg->getvehicleType() == simpledualmotorplane) 
  {
    cfg->setpwmPrescaler2(32);
  }
  // ----------- IMPORTANT!! --------------
  // Motor 1 always runs @ 984Hz PWM frequency and can't be changed, because timers 0 an 1 are in use for other things!
  // Motor 2 (pin 3) can be changed to the following PWM frequencies: 32 = 984Hz, 8 = 3936Hz, 1 = 31488Hz
  setPWMPrescaler(3, cfg->getpwmPrescaler2()); // pin 3 is hardcoded, because we can't change all others anyway
}


MC34933* getMotorptr(motor_t motor)
{
  if(motor==1)
  {
    return &Motor1;
  }
  else
  {
    return &Motor2;
  }
  
}

//
// =======================================================================================================
// DRIVE MOTORS CAR (for cars, one motor for driving, one for steering)
// =======================================================================================================
//

void drive()
{
  vehicleConfig* cfg = getConfigptr();

  switch(cfg->getvehicleType())
  {
    case car:
    {
      driveMotorsCar();

    }
    break;

    case carwithMRSC:
    {
      mrsc(); // Car with MSRC stabilty control
    }
    break;

    case forklift:
    {
      driveMotorsForklift(); // Forklift
    } 
    break;

    case balancingthing:
    {
      balancing(); // Self balancing robot
    }
    break;

    default:
    {
      driveMotorsSteering(); // Caterpillar and half caterpillar vecicles
    }
    break;


  }


}


void driveMotorsCar() {

  vehicleConfig* cfg = getConfigptr();
  RcData* data = getDataptr();
  ackPayload* payload = getPayloadptr();

  int maxPWM;
  byte maxAcceleration;

  // Speed limitation (max. is 255)
  if (data->mode1) {
    maxPWM = cfg->getmaxPWMlimited(); // Limited
  } else {
    maxPWM = cfg->getmaxPWMfull(); // Full
  }

  if (!payload->batteryOk && cfg->getliPo()) data->axis3 = 50; // Stop the vehicle, if the battery is empty!

  // Acceleration & deceleration limitation (ms per 1 step input signal change)
  if (data->mode2) {
    maxAcceleration = cfg->getmaxAccelerationLimited(); // Limited
  } else {
    maxAcceleration = cfg->getmaxAccelerationFull(); // Full
  }

  // ***************** Note! The ramptime is intended to protect the gearbox! *******************
  // SYNTAX: Input value, max PWM, ramptime in ms per 1 PWM increment
  // false = brake in neutral position inactive

  if (!cfg->getHP()) { // Two channel version: ----
    if (Motor1.drive(data->axis3, cfg->getminPWM(), maxPWM, maxAcceleration, true) ) { // The drive motor (function returns true, if not in neutral)
      UpdatemillisLightOff(); // Reset the headlight delay timer, if the vehicle is driving!
    }
    if (cfg->getvehicleType() != 5) { // If not car with MSRC stabilty control
      Motor2.drive(data->axis1, 0, cfg->getsteeringTorque(), 0, false); // The steering motor (if the original steering motor is reused instead of a servo)
    }
  }
  else { // High Power "HP" version. Motor 2 is the driving motor, no motor 1: ----
    if (Motor2.drive(data->axis3, cfg->getminPWM(), maxPWM, maxAcceleration, true) ) { // The drive motor (function returns true, if not in neutral)
      UpdatemillisLightOff(); // Reset the headlight delay timer, if the vehicle is driving!
    }
  }
}

//
// =======================================================================================================
// DRIVE MOTORS FORKLIFT (for forklifts, one motor for driving, one for lifting)
// =======================================================================================================
//

void driveMotorsForklift() {

  vehicleConfig* cfg = getConfigptr();
  RcData* data = getDataptr();
  ackPayload* payload = getPayloadptr();

  int maxPWM;
  byte maxAcceleration;

  // Speed limitation (max. is 255)
  if (data->mode1) {
    maxPWM = cfg->getmaxPWMlimited(); // Limited
  } else {
    maxPWM = cfg->getmaxPWMfull(); // Full
  }

  if (!payload->batteryOk && cfg->getliPo()) data->axis3 = 50; // Stop the vehicle, if the battery is empty!

  // Acceleration & deceleration limitation (ms per 1 step input signal change)
  if (data->mode2) {
    maxAcceleration = cfg->getmaxAccelerationLimited(); // Limited
  } else {
    maxAcceleration = cfg->getmaxAccelerationFull(); // Full
  }

  // ***************** Note! The ramptime is intended to protect the gearbox! *******************
  // SYNTAX: Input value, max PWM, ramptime in ms per 1 PWM increment
  // false = brake in neutral position inactive


  if (Motor1.drive(data->axis3, cfg->getminPWM(), maxPWM, maxAcceleration, true) ) { // The drive motor (function returns true, if not in neutral)
    UpdatemillisLightOff(); // Reset the headlight delay timer, if the vehicle is driving!
  }
  Motor2.drive(data->axis2, 0, cfg->getsteeringTorque(), 0, false); // The fork lifting motor (the steering is driven by servo 1)
}

//
// =======================================================================================================
// "STEERING" MOTOR DRIVING FUNCTION (throttle & steering overlay for caterpillar vehicles)
// =======================================================================================================
//

bool getIsDriving()
{
  return isDriving;
}

void setIsDriving(bool driveing)
{
  isDriving = driveing;
}


void driveMotorsSteering() {

  int pwm[2];

  vehicleConfig* cfg = getConfigptr();
  RcData* data = getDataptr();
  ackPayload* payload = getPayloadptr();

  // The steering overlay
  int steeringFactorLeft=0;
  int steeringFactorRight=0;
  int steeringFactorLeft2=0;
  int steeringFactorRight2 =0;

  // The input signal range
  const int servoMin = 0;
  const int servoMax = 100;
  const int servoNeutralMin = 48;
  const int servoNeutralMax = 52;

  // Compute steering overlay:
  // The steering signal is channel 1 = data.axis1
  // 100% = wheel spins with 100% of the requested speed forward
  // -100% = wheel spins with 100% of the requested speed backward
  if (data->axis1 <= servoNeutralMin) {
    steeringFactorLeft = map(data->axis1, servoMin, servoNeutralMin, 0, 100);
    steeringFactorLeft = constrain(steeringFactorLeft, 0, 100);
  }
  else {
    steeringFactorLeft = 100;
  }

  if (data->axis1 >= servoNeutralMax) {
    steeringFactorRight = map(data->axis1, servoMax, servoNeutralMax, 0, 100);
    steeringFactorRight = constrain(steeringFactorRight, 0, 100);
  }
  else {
    steeringFactorRight = 100;
  }

  // Nonlinear steering overlay correction
  if (cfg->getvehicleType() == 6) {
    steeringFactorLeft2 = reMap(curveThrust, steeringFactorLeft); // Differential thrust mode
    steeringFactorRight2 = reMap(curveThrust, steeringFactorRight);
    data->axis3 = constrain(data->axis3, 50, 100); // reverse locked!
  }
  if (cfg->getvehicleType() == 2) {
    steeringFactorLeft2 = reMap(curveFull, steeringFactorLeft); // Caterpillar mode
    steeringFactorRight2 = reMap(curveFull, steeringFactorRight);
  }
  if (cfg->getvehicleType() == 1) {
    steeringFactorLeft2 = reMap(curvevhctype_semi, steeringFactorLeft); // vhctype_semi caterpillar mode
    steeringFactorRight2 = reMap(curvevhctype_semi, steeringFactorRight);
  }

  // Drive caterpillar motors
  // The throttle signal (for both caterpillars) is channel 3 = data.axis3
  // -100 to 100%
  pwm[0] = map(data->axis3, servoMin, servoMax, 100, -100) * steeringFactorRight2 / 100;
  pwm[1] = map(data->axis3, servoMin, servoMax, 100, -100) * steeringFactorLeft2 / 100;

  pwm[0] = map(pwm[0], 100, -100, 100, 0); // convert -100 to 100% to 0-100 for motor control
  pwm[1] = map(pwm[1], 100, -100, 100, 0);

  if (!payload->batteryOk && cfg->getliPo()) { // Both motors off, if LiPo battery is empty!
    pwm[0] = 0;
    pwm[1] = 0;
  }

  Motor1.drive(pwm[0], 0, 255, 0, false); // left caterpillar, 0ms ramp!
  Motor2.drive(pwm[1], 0, 255, 0, false); // right caterpillar

  if (pwm[0] < 40 || pwm[0] > 60 || pwm[1] < 40 || pwm[1] > 60) {
    UpdatemillisLightOff(); // Reset the headlight delay timer, if the vehicle is driving!
  }
}