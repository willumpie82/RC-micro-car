#ifndef MOTORSH
#define MOTORSH

#include "../lib/MC34933/MC34933.h"  // https://github.com/willumpie82/MC34933

typedef enum motor_e
{
    MOTOR1,
    MOTOR2
}motor_t;


// Motors
bool getIsDriving();
void setIsDriving(bool driveing);

void setupMotors();
void driveMotorsCar();
void driveMotorsForklift();
void driveMotorsSteering();
void drive();

MC34933* getMotorptr(motor_t motor);

#endif //MOTORSH