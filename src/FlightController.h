#ifndef __fc_h__
#define __fc_h__

#include "lowPassFilter.h"
#include "Types.h"
#include "MPU6500.h"
#include "IMU.h"
#include "PID.h"
#include "Timer.h"
#include <Servo.h>

#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in Âµs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in Âµs



class flight_controller_t{
public:
  angle_t angle;
  mpu6500_t mpu6500;
  sensor_t mag;
  IMU_t imu;
  

  pid_control_t pidRoll;
  pid_control_t pidPitch;
  pid_control_t pidYaw;

  bool armed;
  bool active;
  double motor[4];
  uint16_t throttle;
  
  double dt;
  Timer deltaTimer;

  angle_t targetAngles;
  
  void init();
  void stabilize();
  void setThrottle(uint8_t new_throttle);
  void updateAngles();
  void setThrottle2(uint16_t new_throttle);
  void setThrottleAbs(uint8_t new_throttle);
  flight_controller_t();
  void writeToMotors();

private:
  Servo esc1;
  Servo esc2;
  Servo esc3;
  Servo esc4;
};

#endif




