#include "Types.h"

#ifndef __mpu6500_h__
#define __mpu6500_h__

class mpu6500_t{

  public:
    sensorRaw_t acc; // raw data -32768 to 3276
    sensorRaw_t gyro; // raw data -32768 to 3276
    
    sensor_t accBodyFrame; // data +- 8g
    angle_t gyroRate; // data +- 2000 deg/s
    
    float accScaleFactor;
    float gyroScaleFactor;

    sensor_t gyroZero;
    sensor_t accZero;

    void init();
    bool dataReady();
    void updateData();
    //private
    int readRegister(uint8_t reg, uint8_t nBytes, uint8_t * i2cBuffer);
    int readRegister(uint8_t reg, uint8_t nBytes, uint16_t * i2cBuffer);
    int writeRegister(uint8_t reg, uint8_t * i2cBuffer, uint8_t nBytes);
    bool calibrateGyro();
    bool calibrateAcc();
    bool calibrateSensor(sensor_t *zeroValues, uint8_t regAddr, uint8_t maxDifference);
    void fixBoardOrientation(sensorRaw_t * sensorValues);
    void fixBoardOrientation(sensor_t * sensorValues);
    bool checkMinMax(int32_t * sensorBuffer, uint16_t bufferLength, uint8_t maxDifference);
};



#endif




