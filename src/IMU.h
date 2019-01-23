#ifndef __imu_h__
#define __imu_h__

#include "MPU6500.h"
#include "lowPassFilter.h"

//#define DEG_TO_RAD  0.017453292519943295769236907684886f
//#define RAD_TO_DEG  57.295779513082320876798154814105f

class IMU_t{
public:
  void getAngles(mpu6500_t * mpu6500, sensor_t * mag, angle_t * angle, double dt );
  void init(sensor_t accZero, sensor_t gyroZero);
  float gyro_cmpf_factor;
  IMU_t();
  
private:
  void rotateV(sensor_t * v, angle_t * angle);
  float calculateHeading(angle_t * angle, sensor_t * mag);
  
  lowPassFilter_t acc_low_pass[3];

  lowPassFilter_t orientation_low_pass[3];
};

#endif




