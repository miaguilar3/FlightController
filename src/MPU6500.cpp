#include "Wire.h"
#include "Arduino.h"
#include "MPU6500.h"

#define MPU6500_ADDRESS                     0x68
#define MPU6500_WHO_AM_I                    0x75
#define MPU6500_WHO_AM_I_ID                 0x70

#define MPU6500_SMPLRT_DIV                  0x19
#define MPU6500_INT_PIN_CFG                 0x37
#define MPU6500_INT_ENABLE                  0x38
#define MPU6500_ACCEL_XOUT_H                0x3B
#define MPU6500_GYRO_XOUT_H                 0x43
#define MPU6500_PWR_MGMT_1                  0x6B
#define MPU6500_WHO_AM_I                    0x75

#define MPU6500_GYRO_SCALE_FACTOR_2000      16.4f
#define MPU6500_ACC_SCALE_FACTOR_8          4096.0f

#define MPU6500_INT_PIN                     17



void mpu6500_t::init(){
  Serial.println("mpu6500 init");
  //char i2cBuffer[5];
  char i2cBuffer[5];
  char i2cRecvBuffer[5];
  
  // initialize a faster i2c ?
  Wire.begin();
  Serial.println("wire begin");
  // look for MPU6050
  readRegister(MPU6500_WHO_AM_I, 1, (uint8_t *)i2cBuffer);
  Serial.println("read register");
  if(i2cBuffer[0] == 0x68){ 
    Serial.println("MPU6050 found"); 
    }
  else{ 
    Serial.println("MPU6050 not found"); 
    while(1); // can't continue without MPU6050
  }

  // Reset device, this resets all internal registers to their default values
  /*readRegister(MPU6500_PWR_MGMT_1, 1, i2cBuffer );
  i2cBuffer[0] = (1 << 7);
  writeRegister(MPU6500_PWR_MGMT_1, i2cBuffer, 1); // doesn't work
  do{
    readRegister(MPU6500_PWR_MGMT_1, 1, i2cRecvBuffer);// use its own recvbuffer
  } while( i2cRecvBuffer[0] & (1 << 7) );
  */
  i2cBuffer[0] = (1 << 0);  
  writeRegister(MPU6500_PWR_MGMT_1, (uint8_t *)i2cBuffer, 1);
  //readRegister(MPU6500_PWR_MGMT_1, 1, i2cRecvBuffer);
  
  i2cBuffer[0] = 0; // Set the sample rate to 1kHz - 1kHz/(1+0) = 1kHz
  i2cBuffer[1] = 0x03; // Disable FSYNC and set 41 Hz Gyro filtering, 1 KHz sampling
  i2cBuffer[2] = 3 << 3; // Set Gyro Full Scale Range to +-2000deg/s
  i2cBuffer[3] = 2 << 3; // Set Accelerometer Full Scale Range to +-8g
  i2cBuffer[4] = 0x03; // 41 Hz Acc filtering
  
  writeRegister(MPU6500_SMPLRT_DIV, (uint8_t *)i2cBuffer, 5);

  // this.gyro...
  gyroScaleFactor = MPU6500_GYRO_SCALE_FACTOR_2000;
  accScaleFactor = MPU6500_ACC_SCALE_FACTOR_8;

  i2cBuffer[0] = (1 << 5) | (1 << 4) | (1 << 1); // Enable LATCH_INT_EN, INT_ANYRD_2CLEAR and BYPASS_EN
                                                   // When this bit is equal to 1, the INT pin is held high until the interrupt is cleared
                                                   // When this bit is equal to 1, interrupt status is cleared if any read operation is performed
                                                   // When asserted, the I2C_MASTER interface pins (ES_CL and ES_DA) will go into 'bypass mode' when the I2C master interface is disabled
                       
  writeRegister(MPU6500_INT_PIN_CFG, (uint8_t *)i2cBuffer, 1); 
  //readRegister(MPU6500_INT_PIN_CFG, 1, i2cRecvBuffer);
  
  i2cBuffer[0] = (1 << 0); // Enable RAW_RDY_EN - When set to 1, Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin
  writeRegister(MPU6500_INT_ENABLE, (uint8_t *)i2cBuffer, 1);
  //readRegister(MPU6500_INT_ENABLE, 1, i2cRecvBuffer);
  
  // Set INT input pin
  pinMode(MPU6500_INT_PIN, INPUT);
  delay(100); // Wait for sensor to stabilize

  /*while(calibrateGyro()){
    
  }*/

  // gonna have to move this
  // save in eeprom so can take off unlevel surface
  /*while(calibrateAcc()){
    
  }*/
  // need to init accBodyFrame to use in IMU.c
  updateData();
  for(uint8_t axis = 0; axis < 3; axis++){
    accBodyFrame.data[axis] = (float) acc.data[axis] / accScaleFactor;
  }
}

// return errors, pass buff
int mpu6500_t::readRegister(uint8_t reg, uint8_t nBytes, uint8_t * i2cBuffer){
  
  int flag = -1;

  // send register address to MPU6050 through I2c
  Wire.beginTransmission(MPU6500_ADDRESS);
  Wire.write(reg);
  flag = Wire.endTransmission();
  if(flag != 0) { Serial.println("Error sending I2c data"); return 1;}

  Wire.beginTransmission(MPU6500_ADDRESS);
  // request response from MPU6050
  // expecting value of register at address just sent through I2c
  Wire.requestFrom(MPU6500_ADDRESS, nBytes);
  //delay(100);
  uint8_t i = 0;
  while (Wire.available() && i < nBytes) { // slave may send less than requested
    i2cBuffer[i] = Wire.read(); // receive a byte as character
    //Serial.println(i2cBuffer[i], HEX);         // print the character
    i++;
  }
  Wire.endTransmission();
  if(i != nBytes){ Serial.println("Error"); return 1; }
  else{ return 0; } 
  
}

int mpu6500_t::writeRegister(uint8_t reg, uint8_t * i2cBuffer, uint8_t nBytes){
  
  int flag = -1;
  //char sendBuf[] = { reg, i2cBuffer[0], i2cBuffer[1], i2cBuffer[2], i2cBuffer[3], i2cBuffer[4] };

  // send register address to MPU6050 through I2c
  Wire.beginTransmission(MPU6500_ADDRESS);
  //Wire.write(sendBuf, nBytes + 1);
  
  Wire.write(reg);
  for(int i = 0; i < nBytes; i++){
    Wire.write(i2cBuffer[i]);
  }
  
  flag = Wire.endTransmission();
  if(flag != 0) { Serial.println("Error sending I2c data"); return 1;}
  return 0;
}

bool mpu6500_t::dataReady(){
  //Serial.println(digitalRead(MPU6500_INT_PIN));
  return digitalRead(MPU6500_INT_PIN);
}

bool mpu6500_t::calibrateGyro(){
  byte rcode = calibrateSensor(&gyroZero, MPU6500_GYRO_XOUT_H, 100);
  Serial.print("GyroZeroX = "); Serial.println(gyroZero.axis.X);
  Serial.print("GyroZeroY = "); Serial.println(gyroZero.axis.Y);
  Serial.print("GyroZeroZ = "); Serial.println(gyroZero.axis.Z);
  return rcode;
  
}

bool mpu6500_t::calibrateAcc(){
  byte rcode = calibrateSensor(&accZero, MPU6500_ACCEL_XOUT_H, 100);
  accZero.axis.Z += accScaleFactor; // or plus?
  Serial.print("AccZeroX = "); Serial.println(accZero.axis.X);
  Serial.print("AccZeroY = "); Serial.println(accZero.axis.Y);
  Serial.print("AccZeroZ = "); Serial.println(accZero.axis.Z);
  if(accZero.axis.Z == 0) return 1;
  return rcode;
}

bool mpu6500_t::calibrateSensor(sensor_t *zeroValues, uint8_t regAddr, uint8_t maxDifference){
  //static?
  int32_t sensorBuffer[3][25];
  //static const?
  uint16_t bufferLength = 25;
  uint8_t i2cBuffer[6];

  for(uint8_t i = 0; i < bufferLength; i++){
    while(!dataReady()){
          
    }
    readRegister(regAddr, 6, i2cBuffer);
    sensorBuffer[0][i] =  (int16_t(i2cBuffer[0]) << 8) | (int16_t(i2cBuffer[1]) & 0x00FF ) ;
    sensorBuffer[1][i] =  (int16_t(i2cBuffer[2]) << 8) | (int16_t (i2cBuffer[3]) & 0x00FF );
    sensorBuffer[2][i] =  (int16_t(i2cBuffer[4]) << 8) | (int16_t (i2cBuffer[5]) & 0x00FF );
    /*Serial.print("  GyroX = "); Serial.print(sensorBuffer[0][i]);
    Serial.print("  GyroY = "); Serial.print(sensorBuffer[1][i]);
    Serial.print("  GyroZ = "); Serial.println(sensorBuffer[2][i]);*/
    /*Serial.print("  GyroX = "); Serial.print(sensorBuffer[0][i], HEX);
    Serial.print("  GyroY = "); Serial.print(sensorBuffer[1][i], HEX);
    Serial.print("  GyroZ = "); Serial.println(sensorBuffer[2][i], HEX);*/
    delay(10);
  }

  for(uint8_t axis = 0; axis < 3; axis++){
    if(!checkMinMax(sensorBuffer[axis], bufferLength, maxDifference))
      return 1;
  }

  for(uint8_t j = 0; j < 3; j++){
    for(uint8_t k = 1; k < bufferLength; k++){
      sensorBuffer[j][0] += sensorBuffer[j][k];
    }
    zeroValues->data[j] = (float) sensorBuffer[j][0] / bufferLength;  
  }

  fixBoardOrientation(zeroValues);

  return 0;
}

void mpu6500_t::fixBoardOrientation(sensorRaw_t * sensorValues){
  sensorRaw_t sensorValuesTemp = *sensorValues;
  sensorValues->axis.X = -sensorValuesTemp.axis.Y;
  sensorValues->axis.Y = -sensorValuesTemp.axis.X;
  sensorValues->axis.Z = -sensorValuesTemp.axis.Z;
}

void mpu6500_t::fixBoardOrientation(sensor_t * sensorValues){
  sensor_t sensorValuesTemp = *sensorValues;
  sensorValues->axis.X = -sensorValuesTemp.axis.Y;
  sensorValues->axis.Y = -sensorValuesTemp.axis.X;
  sensorValues->axis.Z = -sensorValuesTemp.axis.Z;
}

// returns true if min mix difference is less than maxDifference
bool mpu6500_t::checkMinMax(int32_t * sensorBuffer, uint16_t bufferLength, uint8_t maxDifference){
  int32_t minVal = sensorBuffer[0];
  int32_t maxVal = sensorBuffer[0];

  for(int i = 0; i < bufferLength; i++){
    if( sensorBuffer[i] < minVal ){
      minVal = sensorBuffer[i];
    }
    if ( sensorBuffer[i] > maxVal ){
      maxVal = sensorBuffer[i];
    }
  }

  return maxVal - minVal < maxDifference;
}

void mpu6500_t::updateData(){
  uint8_t i2cBuffer[14];
  readRegister(MPU6500_ACCEL_XOUT_H, 14, i2cBuffer);
  acc.data[0] = (int16_t(i2cBuffer[0]) << 8) | (int16_t(i2cBuffer[1]) & 0x00FF );
  acc.data[1] = (int16_t(i2cBuffer[2]) << 8) | (int16_t(i2cBuffer[3]) & 0x00FF );
  acc.data[2] = (int16_t(i2cBuffer[4]) << 8) | (int16_t(i2cBuffer[5]) & 0x00FF );

  gyro.data[0] = (int16_t(i2cBuffer[8]) << 8) | (int16_t(i2cBuffer[9]) & 0x00FF );
  gyro.data[1] = (int16_t(i2cBuffer[10]) << 8) | (int16_t(i2cBuffer[11]) & 0x00FF );
  gyro.data[2] = (int16_t(i2cBuffer[12]) << 8) | (int16_t(i2cBuffer[13]) & 0x00FF );

  fixBoardOrientation(&gyro);
  fixBoardOrientation(&acc);

  for(uint8_t axis = 0; axis < 3; axis++){
    acc.data[axis] -= accZero.data[axis]; // subtract accelerometer zero values
    gyro.data[axis] -= gyroZero.data[axis]; // subtract gyro zero values
    gyroRate.data[axis] = (float)gyro.data[axis] / gyroScaleFactor; // convert to deg/s
    
    // going to need to be moved. check magnitude first before converting
    accBodyFrame.data[axis] = (float) acc.data[axis] / accScaleFactor;
  }
}





