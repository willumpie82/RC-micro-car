#ifndef MOTORSH
#define MOTORSH

#include "MC34933.h" 

typedef enum motor_e
{
    MOTOR1,
    MOTOR2
}motor_t;

// motor wrapper functions
void motordrive(motor_t motor, int controlValue, int minPWM, int maxPWM, int rampTime, bool neutralBrake);
bool motorBrakeActive(motor_t motor);

// Motors
bool getIsDriving();
void setIsDriving(bool driveing);

void setupMotors();
void driveMotorsCar();
void driveMotorsForklift();
void driveMotorsSteering();
void drive();



//MC34933* getMotorptr(motor_t motor);

#endif //MOTORSH