#include <Arduino.h>
#include "balancing.h"

#include "vehicleConfig.h"
#include "MPU6050.h"
#include "radio.h"
#include "PID_v1.h" // https://github.com/br3ttb/Arduino-PID-Library/
#include "motors.h"

int speedPot;

int speedAveraged;


//Define PID Variables
double speedTarget, speedMeasured, speedOutput;
double angleTarget, angleMeasured, angleOutput;

double speedAverage;

// PID controllers (you may have to change their parameters in balancing() )
//Kp: proportional (instantly), Ki: integral (slow, precise), Kd: deriative (speed of difference)
PID speedPid(&speedMeasured, &speedOutput, &speedTarget, 0.0, 0.0, 0.0, DIRECT); // Speed: outer, slow control loop
PID anglePid(&angleMeasured, &angleOutput, &angleTarget, 0.0, 0.0, 0.0, DIRECT); // Angle: inner, fast control loop


double getAngleOut( void )
{
  return angleOutput;
}
//
// =======================================================================================================
// PID SETUP
// =======================================================================================================
//

void setupPid() {

  // Speed control loop
  speedPid.SetSampleTime(8); // calcualte every 4ms = 250Hz
  speedPid.SetOutputLimits(-33, 33); // output range from -33 to 33 (same as in balancing() )
  speedPid.SetMode(AUTOMATIC);

  // Angle control loop
  anglePid.SetSampleTime(8); // calcualte every 4ms = 250Hz
  anglePid.SetOutputLimits(-43, 43); // output range from -43 to 43 for motor
  anglePid.SetMode(AUTOMATIC);
}


//
// =======================================================================================================
// BALANCING CALCULATIONS
// =======================================================================================================
//

void balancing() {

  vehicleConfig* cfg = getConfigptr();
  RcData* data = getDataptr();
  // Read sensor data
  readMpu6050Data();

  angleMeasured = Mpu6050_getangle_pitch() - cfg->gettiltCalibration();

  // Read speed pot with 0.2s fader
  static unsigned long lastPot;
  if (millis() - lastPot >= 40) { // 40ms
    lastPot = millis();
    speedPot = (speedPot * 4 + data->axis3) / 5; // 1:5
  }

  // PID Parameters (Test)
  double speedKp = 0.9, speedKi = 0.03, speedKd = 0.0;
  double angleKp = data->pot / 8.0, angleKi = 25.0, angleKd = 0.12; // /You need to connect a potentiometer to the transmitter analog input A6

  // PID Parameters (Working)
  //double speedKp = 0.9, speedKi = 0.03, speedKd = 0.0;
  //double angleKp = data.pot1 / 8.0, angleKi = 25.0, angleKd = 0.12; // You need to connect a potentiometer to the transmitter analog input A6

  // Speed PID controller (important to protect the robot from falling over at full motor rpm!)
  speedTarget = ((float)speedPot - 50.0) / 1.51; // (100 - 50) / 1.51 = Range of about +/- 33 (same as in setupPid() !)
  speedMeasured = speedAveraged * 1.3; //angleOutput; // 43 / 33 = 1.3
  speedPid.SetTunings(speedKp, speedKi, speedKd);
  speedPid.Compute();

  // Angle PID controller
  angleTarget = speedOutput / -8.25; // 33.0 (from above) / 8.25 = Range of about +/- 4.0° tilt angle
  //  angleTarget = (speedPot - 50) / -12.5; // 50 / 12.5 = Range of about +/- 4.0° tilt angle
  anglePid.SetTunings(angleKp, angleKi, angleKd);
  anglePid.Compute();

  // Send the calculated values to the motors
  driveMotorsBalancing();

  // Display PID variables on the transmitter OLED (for debugging only, comment out the corresponding variables in checkBattery() in this case)
  //loopDuration(); // compute the loop time
  //payload.vcc = loopTime;
  //payload.vcc = angleMeasured;
  //payload.vcc = speedTarget;
  //payload.batteryVoltage = speedOutput;
  //payload.batteryVoltage = speedMeasured;
}

//
// =======================================================================================================
// MRSC (MICRO RC STABILITY CONTROL) CALCULATIONS
// =======================================================================================================
// For cars with stability control (steering overlay depending on gyro yaw rate)

void mrsc() {

  vehicleConfig* cfg = getConfigptr();
  RcData* data = getDataptr();

  // Read sensor data
  readMpu6050Data();

  // If the MRSC gain is a fixed value, read it!
#ifdef MRSC_FIXED
  data.pot1 = mrscGain;
#endif

  // Compute steering compensation overlay
  int turnRateSetPoint = data->axis1 - 50;  // turnRateSetPoint = steering angle (0 to 100) - 50 = -50 to 50
  int turnRateMeasured = Mpu6050_getyaw_rate() * abs(data->axis3 - 50); // degrees/s * speed
  int steeringAngle = turnRateSetPoint + (turnRateMeasured * data->pot / 100);  // Compensation depending on the pot value

  steeringAngle = constrain (steeringAngle, -50, 50); // range = -50 to 50

  // Control steering servo (MRSC mode only)
#ifdef CONFIG_HAS_SERVO
  servo1.write(map(steeringAngle, 50, -50, lim1L, lim1R) ); // 45 - 135°
#endif //CONFIG_HAS_SERVO
  // Control motor 2 (steering, not on "High Power" board type)
  if (!cfg->getHP()) {
    motordrive(MOTOR2, (steeringAngle + 50), 0, cfg->getsteeringTorque(), 0, false); // The steering motor (if the original steering motor is reused instead of a servo)
  }

  // Control motors
  driveMotorsCar();

}


void driveMotorsBalancing() 
{
  vehicleConfig* cfg = getConfigptr();
  RcData* data = getDataptr();

  // The steering overlay is in degrees per second, controlled by the MPU 6050 yaw rate and yoystick axis 1
  int steering = ((data->axis1 - 50) / 7) - Mpu6050_getyaw_rate(); // -50 to 50 / 8 = 7°/s - yaw_rate
  steering = constrain(steering, -7, 7);
  int speed = angleOutput + 50;

  // Calculate averaged motor power (speed) for speed controller feedback
  static unsigned long lastSpeed;
  if (millis() - lastSpeed >= 8) {  // 8ms
    speedAveraged = (speedAveraged * 3.0 + angleOutput) / 4.0; // 1:4 (1:8)
  }

  speed = constrain(speed, 7, 93); // same range as in setupPID() + 50 offset from above!

  if (angleMeasured > -20.0 && angleMeasured < 20.0) { // Only drive motors, if robot stands upright
    motordrive(MOTOR1, speed - steering, cfg->getminPWM(), cfg->getmaxPWMfull(), 0, false); // left caterpillar, 0ms ramp! 50 = neutral!
    motordrive(MOTOR2, speed + steering, cfg->getminPWM(), cfg->getmaxPWMfull(), 0, false); // right caterpillar
  }
  else { // keep motors off
    motordrive(MOTOR1, 50, cfg->getminPWM(), cfg->getmaxPWMfull(), 0, false); // left caterpillar, 0ms ramp!
    motordrive(MOTOR2, 50, cfg->getminPWM(), cfg->getmaxPWMfull(), 0, false); // right caterpillar
  }
}