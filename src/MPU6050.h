#ifndef MPU6050H
#define MPU6050H

//#include "servos.h"

void writeDebug();
void readMpu6050Data();
void setupMpu6050();

float Mpu6050_getangle_pitch( void );
float Mpu6050_getyaw_rate( void );



#endif //MPU6050H