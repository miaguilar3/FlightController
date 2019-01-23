#include "IMU.h"
#include "Arduino.h"
#include "LowPassFilter.h"
#include "math.h"

IMU_t::IMU_t(){

}

void IMU_t::getAngles(mpu6500_t * mpu6500, sensor_t * mag, angle_t * angle, double dt ){
  gyro_cmpf_factor = 600.0f; // need to be able to update
  float inv_gyro_cmpf_factor = 1.0f / (gyro_cmpf_factor + 1.0f);

  sensor_t gyro;
  sensor_t accLPF;
  float accMagSquare = 0; // hold magnitude to check if it can possibly be gravity vector

  for(uint8_t axis = 0; axis < 3; axis++){
    gyro.data[axis] = mpu6500->gyroRate.data[axis] * DEG_TO_RAD; // convert from deg/s to rad 
    accLPF.data[axis] = acc_low_pass[axis].applyLowPass(mpu6500->acc.data[axis], dt);
    accMagSquare += accLPF.data[axis] * accLPF.data[axis];
  }

  angle_t deltaAngle;
  deltaAngle.axis.roll = gyro.axis.X * dt;
  deltaAngle.axis.pitch = gyro.axis.Y * dt;
  deltaAngle.axis.yaw = gyro.axis.Z * dt;

  rotateV(&mpu6500->accBodyFrame, &deltaAngle);

  // check if we have possible gravity vector
  accMagSquare /= (mpu6500->accScaleFactor * mpu6500->accScaleFactor);
  if(0.72f < accMagSquare && accMagSquare < 1.32f){
    for(uint8_t axis = 0; axis < 3; axis++){
      // complimentary filter
      mpu6500->accBodyFrame.data[axis] = (mpu6500->accBodyFrame.data[axis] * gyro_cmpf_factor + accLPF.data[axis]/mpu6500->accScaleFactor) / inv_gyro_cmpf_factor;
    }
  }

  angle->axis.roll = atan2(-mpu6500->accBodyFrame.axis.Y, sqrt(mpu6500->accBodyFrame.axis.X * mpu6500->accBodyFrame.axis.X + mpu6500->accBodyFrame.axis.Z * mpu6500->accBodyFrame.axis.Z));
  angle->axis.pitch = atan2(mpu6500->accBodyFrame.axis.X, -mpu6500->accBodyFrame.axis.Z);

  //float gyro_cmpfm_factor = 250.0f;
  //float inv_gyro_cmpfm_factor = (1.0f / (250.0f + 1.0f));

  rotateV(mag, &deltaAngle);
  angle->axis.yaw = calculateHeading(angle, mag);

  angle->axis.roll = angle->axis.roll * RAD_TO_DEG;
  angle->axis.pitch = angle->axis.pitch * RAD_TO_DEG;

  //angle->axis.roll = orientation_low_pass[0].applyLowPass(angle->axis.roll, dt);
  //angle->axis.pitch = orientation_low_pass[1].applyLowPass(angle->axis.pitch, dt);
  //angle->axis.yaw = orientation_low_pass[2].applyLowPass(angle->axis.yaw, dt);
}

float IMU_t::calculateHeading(angle_t * angle, sensor_t * mag){
  float cosx = cosf(angle->axis.roll);
  float sinx = sinf(angle->axis.roll);
  float cosy = cosf(angle->axis.pitch);
  float siny = sinf(angle->axis.pitch);

  float Bfy = mag->axis.Z * sinx - mag->axis.Y * cosx;
  float Bfx = mag->axis.X * cosy + mag->axis.Y * siny * sinx + mag->axis.Z * siny * cosx;

  float heading = atan2(Bfy, Bfx) * RAD_TO_DEG;

  if(heading < 0) { heading += 360; };
  return heading;
}


void IMU_t::rotateV(sensor_t * v, angle_t * angle){
  sensor_t v_tmp = *v;
  
  float cosx = cosf(angle->axis.roll);
  float sinx = sinf(angle->axis.roll);
  float cosy = cosf(angle->axis.pitch);
  float siny = sinf(angle->axis.pitch);
  float cosz = cosf(angle->axis.yaw);
  float sinz = sinf(angle->axis.yaw);

  float coszcosx = cosz * cosx;
  float sinzcosx = sinz * cosx;
  float coszsinx = sinx * cosz;
  float sinzsinx = sinx * sinz;

  float mat[3][3];
  mat[0][0] = cosz * cosy;
  mat[0][1] = -cosy * sinz;
  mat[0][2] = siny;
  mat[1][0] = sinzcosx +(coszsinx * siny); 
  mat[1][1] = coszcosx - (sinzsinx * siny);
  mat[1][2] = -sinx * cosy;
  mat[2][0] = (sinzsinx) - (coszcosx * siny);
  mat[2][1] = (coszsinx) + (sinzcosx * siny);
  mat[2][2] = cosy * cosx;

  v->axis.X = v_tmp.axis.X * mat[0][0] + v_tmp.axis.Y * mat[1][0] + v_tmp.axis.Z * mat[2][0]; 
  v->axis.Y = v_tmp.axis.X * mat[0][1] + v_tmp.axis.Y * mat[1][1] + v_tmp.axis.Z * mat[2][1];
  v->axis.Z = v_tmp.axis.X * mat[0][2] + v_tmp.axis.Y * mat[1][2] + v_tmp.axis.Z * mat[2][2];
}

void IMU_t::init(sensor_t accZero, sensor_t gyroZero){
  acc_low_pass[0].Fc = 53.05;
  acc_low_pass[1].Fc = 53.05;
  acc_low_pass[2].Fc = 53.05;

  orientation_low_pass[0].Fc = 53.05 / 2;
  orientation_low_pass[1].Fc = 53.05 / 2;
  orientation_low_pass[2].Fc = 53.05 / 2;
  
  acc_low_pass[0].prevOut = accZero.data[0];
  acc_low_pass[1].prevOut = accZero.data[1];
  acc_low_pass[2].prevOut = accZero.data[2];

  orientation_low_pass[0].prevOut = 0.0;
  orientation_low_pass[1].prevOut = 0.0;
  orientation_low_pass[2].prevOut = 0.0;
}





