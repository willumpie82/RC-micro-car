#include <Arduino.h>
#include "servos.h"

//
// =======================================================================================================
// WRITE SERVO POSITIONS
// =======================================================================================================
//

void setupServos( void )
{
#ifdef CONFIG_HAS_SERVO
  servo1.attach(A0);
  if (!tailLights) servo2.attach(A1);
  if (!engineSound && !toneOut) servo3.attach(A2);
  if (!beacons) servo4.attach(A3);
#endif //CONFIG_HAS_SERVO
}

void writeServos() 
{
#ifdef CONFIG_HAS_SERVO
  // Aileron or Steering
  if (vehicleType != 5) { // If not car with MSRC stabilty control
    servo1.write(map(data.axis1, 100, 0, lim1L, lim1R) ); // 45 - 135°
  }

  // Elevator or shifting gearbox actuator
#ifdef TWO_SPEED_GEARBOX // Shifting gearbox mode, controlled by "Mode 1" button
  if (!tailLights) {
    if (data.mode1)servo2.write(lim2L);
    else servo2.write(lim2R);
  }

#else
#ifdef THREE_SPEED_GEARBOX // Shifting gearbox mode, controlled by 3 position switch
  if (!tailLights) {
    if (data.axis2 < 10)servo2.write(lim2R);
    else if (data.axis2 > 90)servo2.write(lim2L);
    else servo2.write(lim2C);
  }

#else // Servo controlled by joystick CH2
  if (!tailLights) servo2.write(map(data.axis2, 100, 0, lim2L, lim2R) ); // 45 - 135°
#endif
#endif

  // Throttle (for ESC control, if you don't use the internal TB6612FNG motor driver)
  if (data.mode1) { // limited speed!
    servo3.write(map(data.axis3, 100, 0, lim3Llow, lim3Rlow ) ); // less than +/- 45°
  }
  else { // full speed!
    servo3.write(map(data.axis3, 100, 0, lim3L, lim3R) ); // 45 - 135°
  }

  // Rudder or trailer unlock actuator
#ifdef TRACTOR_TRAILER_UNLOCK // Tractor trailer unlocking, controlled by "Momentary 1" ("Back / Pulse") button
  if (!beacons && !potentiometer1) {
    if (data.momentary1)servo4.write(lim4L);
    else servo4.write(lim4R);
  }

#else // Servo controlled by joystick CH4 
  if (!potentiometer1) { // Servo 4 controlled by CH4
    if (!beacons) servo4.write(map(data.axis4, 100, 0, lim4L, lim4R) ); // 45 - 135°
  }
  else { // Servo 4 controlled by transmitter potentiometer knob
    if (!beacons) servo4.write(map(data.pot1, 0, 100, 45, 135) ); // 45 - 135°
  }
#endif

  // Axis 2 on the joystick switches engine sound on servo channel 3 on and off!
  if (engineSound) {
    if (data.axis2 > 80) {
      servo3.attach(A2); // Enable servo 3 pulse
    }
    if (data. axis2 < 20) {
      servo3.detach(); // Disable servo 3 pulse = engine off signal for "TheDIYGuy999" engine simulator!
    }
  }
#endif //CONFIG_HAS_SERVO
}