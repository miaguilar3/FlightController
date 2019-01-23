#include "FlightController.h"
#include<Servo.h>
#define LED_PIN A2// 29

flight_controller_t::flight_controller_t(){

  
}

void flight_controller_t::init(){

  digitalWrite(LED_PIN, LOW);
  /*esc1.attach(35, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  esc2.attach(23, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // 
  esc3.attach(30, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // 
  esc4.attach(2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);//*/
  
  esc1.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  esc2.attach(5, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // 
  esc3.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // 
  esc4.attach(10, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);//
  

  esc1.writeMicroseconds(2000);
  esc2.writeMicroseconds(2000);
  esc3.writeMicroseconds(2000);
  esc4.writeMicroseconds(2000);
  delay(5000);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(10000);
  
  
  /**/

  digitalWrite(LED_PIN, HIGH);
  delay(2000);
  armed = false;
  active = false;
  motor[0] = motor[1] = motor[2] = motor[3] = 1000.0;
  throttle = 1000;
  
  pidRoll.init(0.35, 0.13, 0.1, 3.0, 53.05);//usually hold it
  pidPitch.init(0.35, 0.13, 0.1, 3.0, 53.05);// weird way

  pidYaw.init(0.1, 0.13, 0.1, 3.0, 53.05);

  mpu6500.init();

  mpu6500.gyroZero.axis.X = -19.00;
  mpu6500.gyroZero.axis.Y = 73.40;
  mpu6500.gyroZero.axis.Z = 11.96;
  mpu6500.accZero.axis.X = -77.88;
  mpu6500.accZero.axis.Y = 163.60;
  mpu6500.accZero.axis.Z = -171.2;

  //tied down
  /*mpu6500.gyroZero.axis.X = -19.04;
  mpu6500.gyroZero.axis.Y = 73.04;
  mpu6500.gyroZero.axis.Z = -5.04;
  mpu6500.accZero.axis.X = -46.00;
  mpu6500.accZero.axis.Y = 315.96;
  mpu6500.accZero.axis.Z = -224.6;*/
  
  imu.init(mpu6500.accZero, mpu6500.gyroZero);
  mag.data[0] = 1.0;
  mag.data[1] = 0.0;
  mag.data[2] = 0.0;
  targetAngles.axis.roll = 0.0;
  targetAngles.axis.pitch = 0.0;
  targetAngles.axis.yaw = 0.0;

  deltaTimer.init();
}

void flight_controller_t::writeToMotors(){
  float maxVal = motor[0];
  for(int i = 0; i < 4; i++){
    if(motor[i] > maxVal){
      maxVal = motor[i];
    }
  }
  if(maxVal > 2000){ // change here
    for(int j = 0; j < 4; j++){
      motor[j] -= maxVal - 2000; // change here
    }
  }
  for(int j = 0; j < 4; j++){
     if(motor[j] < 1000){
        motor[j] = 1000;
     }
  }

  esc1.writeMicroseconds(static_cast<int>(motor[0]));
  esc2.writeMicroseconds(static_cast<int>(motor[1]));
  esc3.writeMicroseconds(static_cast<int>(motor[2]));
  esc4.writeMicroseconds(static_cast<int>(motor[3]));

  /*Serial.print("Motor0: "); Serial.print(motor[0]);
Serial.print("  Motor1: "); Serial.print(motor[1]);
Serial.print("  Motor2: "); Serial.print(motor[2]);
Serial.print("  Motor3: "); Serial.println(motor[3]);*/
  //analogWrite(9, static_cast<int>(motor[0])); // pb1
  //analogWrite(10, static_cast<int>(motor[1]));// pb2
  //analogWrite(6, static_cast<int>(motor[2]));
  //analogWrite(5, static_cast<int>(motor[3]));
}

void flight_controller_t::updateAngles(){
  if(mpu6500.dataReady()){
      mpu6500.updateData();
      dt = deltaTimer.getElapsedTime() / 1000000.0f;
      deltaTimer.reset();
      imu.getAngles(&mpu6500, &mag, &angle, dt);
  }
}

void flight_controller_t::stabilize(){
  if(active){
      double setPointRoll = targetAngles.axis.roll - angle.axis.roll;
      double setPointPitch = targetAngles.axis.pitch - angle.axis.pitch;
      //setPointYaw = 0 * scaling?
      setPointRoll = setPointRoll * 4.5;
      setPointPitch = setPointPitch * 4.5;
      double rollOut = pidRoll.updatePID(setPointRoll, mpu6500.gyroRate.axis.roll, dt);
      double pitchOut = pidPitch.updatePID(setPointPitch, mpu6500.gyroRate.axis.pitch, dt);

      for(int i = 0; i < 4; i++){
        motor[i] = throttle;
      }

      motor[0] += rollOut;
      motor[1] += rollOut;
      motor[2] -= rollOut;
      motor[3] -= rollOut;

      motor[0] -= pitchOut; // +=
      motor[1] += pitchOut; // -=
      motor[2] -= pitchOut; // +=
      motor[3] += pitchOut; // -=

      writeToMotors();
  }
}


void flight_controller_t::setThrottle(uint8_t new_throttle){
  throttle = motor[0] = motor[1] = motor[2] = motor[3] = 1000.0 + ((1000.0/255.0) * new_throttle) ;
  if(armed)
    writeToMotors();
}

void flight_controller_t::setThrottle2(uint16_t new_throttle){
  throttle = new_throttle;
  if(armed)
    writeToMotors();
}


