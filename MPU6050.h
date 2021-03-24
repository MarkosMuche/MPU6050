#ifndef MPU6050_h
#define MPU6050_h
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class MPU6050
{
  public:
    //constructor
    MPU6050();
    //methods
    void setupSensor();
    float* readSensor(float sensorReturn[6],float dt);
    void resetGyroAngles();
    // fields
float gyroOffset[3]={7.32, 21.42,-71.15};
    const int MPU_ADDRESS=0x68;
    float FREQ=250;
    float GyroConst=65.5;
    float gyroAngle[3];
    float accAngle[2];
    float gyroRate[3];
};
#endif
