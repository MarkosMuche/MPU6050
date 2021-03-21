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
    float* readSensor(float sensorReturn[6]);
    void resetGyroAngles();
    // fields
    float gyroOffset[3];
    const int MPU_ADDRESS=0x68;
    float FREQ;
    float GyroConst;
    float gyroAngle[3];
    float accAngle[2];
    float gyroRate[3];
};
#endif
