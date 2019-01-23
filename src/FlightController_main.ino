#include "Types.h"
#include "MPU6500.h"
#include "IMU.h"
#include "PID.h"
#include "Timer.h"
#include "FlightController.h"
#include "Timer2.h"
#include "GraphFlight.h"

#define LED_PIN A2 //29

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(3000);
  
  Serial.begin(9600);
  Serial.print("setup");
}

enum {
    SET_THROTTLE = 48,
    SEND_ANGLES,
    SEND_MOTOR_SPEEDS,
    SEND_TARGET_ANGLES,
    SEND_PID_CONSTANTS,
    SEND_GYRO_ZEROS,
    SEND_ACC_ZEROS,
    SEND_STATIC,
    SET_PID_CONSTANTS,
    SEND_GRAPH_FLIGHT_DATA,
    SET_ACTIVE,
    DISABLE_ACTIVE,
    ARM,
    UNARM,
    START_GRAPH_FLIGHT,
    END_GRAPH_FLIGHT,

    
    SET_TARGET_ANGLES,
    SET_PID_YAW,
    GET_PID_YAW,
    RESTORE_DEFAULTS,
};

// need to init prevOut
// need to init accBodyFrame
void loop() {
  
  Timer2 bt_timer;
  flight_controller_t flightController;
  graph_flight_t graph_flight_handler;
  
  bt_timer.init();  
  flightController.init();
  graph_flight_handler.init();

  digitalWrite(LED_PIN, LOW);
 
  while(1){
    
    flightController.updateAngles();
    //Serial.print("  Roll: ");Serial.print(flightController.angle.axis.roll);
    //Serial.print("  Pitch: ");Serial.print(flightController.angle.axis.pitch);
    //Serial.print("  Yaw: ");Serial.println(flightController.angle.axis.yaw);

    if(flightController.armed){
      if(flightController.active){
        flightController.stabilize();
      }
  
      if(graph_flight_handler.active){
        graph_flight_handler.gfh_update(&flightController);
      }
    }
    if(bt_timer.getElapsedTime() > 500){
      bt_timer.reset();
      handleBluetooth(&flightController, &graph_flight_handler);
    }
  }
}

void handleBluetooth(flight_controller_t * flightController, graph_flight_t * graph_flight_handler){
  msg_t msg;
  msg.cmd = 0b11111111; msg.len = 0b11111111;
  if(Serial.available() >= 5){// header length
    //Serial.println("serial available");
    if(headerFound()){
      //Serial.println("header found");
      if(getMessage(&msg, sizeof(msg))){
        if(Serial.available() < msg.len){
           // wait or return
        }
        switch(msg.cmd){
          case SET_THROTTLE:
            handleCommandSetThrottle(flightController, msg.len);
            break;
          case SEND_ANGLES:
            handleCommandSendAngles(flightController, msg.len);  
            break;
          case SEND_MOTOR_SPEEDS:
            handleCommandSendMotorSpeeds(flightController, msg.len);    
            break;
          case SEND_TARGET_ANGLES:
            handleCommandSendTargetAngles(flightController, msg.len);
            break;
          case SEND_PID_CONSTANTS:
            handleCommandSendPidConstants(flightController, msg.len);
            break;
          case SEND_GYRO_ZEROS:
            handleCommandSendGyroZeros(flightController, msg.len);
            break;
          case SEND_ACC_ZEROS:
            handleCommandSendAccZeros(flightController, msg.len);
            break;
          case SEND_STATIC:
            // send static data
            handleCommandSendPidConstants(flightController, 0);
            handleCommandSendGyroZeros(flightController, 0);
            handleCommandSendAccZeros(flightController, 0);
            break;
          case SET_TARGET_ANGLES:
            handleCommandSetTargetAngles(flightController, msg.len);
            break;
          case SET_PID_CONSTANTS:
             handleCommandSetPidConstants(flightController, msg.len);
          case SET_ACTIVE:
            handleCommandSetActive(flightController, msg.len);
            break;
          case SEND_GRAPH_FLIGHT_DATA:
            handleCommandSendGraphFlightData(flightController, msg.len, graph_flight_handler);
            break;
          case DISABLE_ACTIVE:
            handleCommandDisableActive(flightController, msg.len);
            break;
          case ARM:
            handleCommandArm(flightController, msg.len);
            break;
          case UNARM:
            handleCommandUnarm(flightController, msg.len);
            break;
          case START_GRAPH_FLIGHT:
            handleCommandStartGraphFlight(flightController, msg.len, graph_flight_handler);
            break;
          
        }
        clearSerialBuffer();
      }
      clearSerialBuffer();
    }
    return;
  }
}

void sendHeader(uint8_t command, uint8_t data_length){
  Serial.print("$S>");
  Serial.print(char(command));
  Serial.print(char(data_length));
}

void sendMessage(uint8_t command, uint8_t msg_data_len, char * msg ){
  uint8_t null_counts = 0;
  char * cp = msg;

  for(int i = 0; i < msg_data_len; i++){
    if(*cp == '\x00') null_counts++;
    cp++;
  }

  // not going to send null chars so subtract them from the message data length
  sendHeader(command, msg_data_len - null_counts);
  
  for(int i = 0; i < msg_data_len; i++){
    if(*msg != '\x00') Serial.print(*msg);
    msg++;
  }
}

void handleCommandSetActive(flight_controller_t * flightController, uint8_t msg_len){
   flightController->active = true;
}

void handleCommandDisableActive(flight_controller_t * flightController, uint8_t msg_len){
   flightController->active = false;
}

void handleCommandArm(flight_controller_t * flightController, uint8_t msg_len){
  digitalWrite(LED_PIN, HIGH);
  flightController->armed = true;
  
}

void handleCommandUnarm(flight_controller_t * flightController, uint8_t msg_len){
  flightController->setThrottle2(1000);
  flightController->armed = false;
  digitalWrite(LED_PIN, LOW);
}

void handleCommandStartGraphFlight(flight_controller_t * flightController, uint8_t msg_len, graph_flight_t * graph_flight_handler){
  if(flightController->armed)
    graph_flight_handler->start(flightController);
}

void handleCommandSendGraphFlightData(flight_controller_t * flightController, uint8_t msg_len, graph_flight_t * graph_flight_handler){
  motor_speed_msg_t msm;
  for(int i = 0; i < GRAPH_SAVE_VALUES; i++){
    sprintf(msm.motor0, "%04d", static_cast<int>(graph_flight_handler->motor1_vals[i]));
    sprintf(msm.motor1, "%04d", static_cast<int>(graph_flight_handler->motor2_vals[i]));
    sprintf(msm.motor2, "%04d", static_cast<int>(graph_flight_handler->motor3_vals[i]));
    sprintf(msm.motor3, "%04d", static_cast<int>(graph_flight_handler->motor4_vals[i]));
    sendMessage(SEND_MOTOR_SPEEDS, sizeof(msm), (char*)&msm);
  }
}

// maybe put some checks to see if values make sense
void handleCommandSetTargetAngles(flight_controller_t * flightController, uint8_t msg_len){
  Serial.println("SET TARGET ANGLES");
  angle_msg_t am;
  //msg length = 18
  uint8_t NUM_ANGLES = 3;
  fillBytes((char * )&am.roll, 6);
  fillBytes((char * )&am.pitch, 6);
  fillBytes((char * )&am.yaw, 6);
  am.roll[6] = NULL;
  am.pitch[6] = NULL;
  am.yaw[6] = NULL;

  float target_roll = (float)(atol((char *)&am.roll)) / 100.0;
  float target_pitch =(float)(atol((char *)&am.pitch)) / 100.0;
  float target_yaw = (float)(atol((char *)&am.yaw)) / 100.0;

  Serial.println(target_roll);
  Serial.println(target_pitch);
  Serial.println(target_yaw);

  flightController->targetAngles.axis.roll = target_roll;
  flightController->targetAngles.axis.pitch = target_pitch;
  flightController->targetAngles.axis.yaw = target_yaw;
  // $S>80+36000+36000+36000
     
}

void handleCommandSetPidConstants(flight_controller_t * flightController, uint8_t msg_len){
  Serial.println("SET PID CONSTANTS");
  pid_constants_msg_t pcm;
  //msg length = 18
  uint8_t NUM_PID_CONSTANTS = 3;
  fillBytes((char * )&pcm.P, 6);
  fillBytes((char * )&pcm.I, 6);
  fillBytes((char * )&pcm.D, 6);
  pcm.P[6] = NULL;
  pcm.I[6] = NULL;
  pcm.D[6] = NULL;

  float Kp = (float)(atol((char *)&pcm.P)) / 100.0;
  float Ki =(float)(atol((char *)&pcm.I)) / 100.0;
  float Kd = (float)(atol((char *)&pcm.D)) / 100.0;

  //Serial.println(Kp);
  //Serial.println(Ki);
  //Serial.println(Kd);

  // check if vals make sense

  flightController->pidRoll.pidValues.Kp = Kp;
  flightController->pidPitch.pidValues.Kp = Kp;
  flightController->pidRoll.pidValues.Ki = Ki;
  flightController->pidPitch.pidValues.Ki = Ki;
  flightController->pidRoll.pidValues.Kd = Kd;
  flightController->pidPitch.pidValues.Kd = Kd;
  // $S>90+00022+00043+00059
}

void handleCommandSendTargetAngles(flight_controller_t * flightController, uint8_t msg_len){
  Serial.println("SEND TARGET ANGLES");
  angle_msg_t am;
  sprintf(am.roll, "%+06d", (int)(flightController->targetAngles.axis.roll * 100));
  sprintf(am.pitch, "%+06d", (int)(flightController->targetAngles.axis.pitch * 100));
  sprintf(am.yaw, "%+06u", (long int)(flightController->targetAngles.axis.yaw * 100));
  sendMessage(SEND_TARGET_ANGLES, sizeof(am), (char*)&am);
}

void handleCommandSendPidConstants(flight_controller_t * flightController, uint8_t msg_len){
  Serial.println("SEND PID CONSTANTS");
  pid_constants_msg_t pcm;
  sprintf(pcm.P, "%+06d", (int)(flightController->pidRoll.pidValues.Kp * 100));
  sprintf(pcm.I, "%+06d", (int)(flightController->pidRoll.pidValues.Ki * 100));
  sprintf(pcm.D, "%+06u", (int)(flightController->pidRoll.pidValues.Kd * 100));
  sendMessage(SEND_PID_CONSTANTS, sizeof(pcm), (char*)&pcm);
}

void handleCommandSendPidTerms(flight_controller_t * flightController, uint8_t msg_len){
  Serial.println("SEND PID TERMS");
  pid_terms_msg_t am;
}

void handleCommandSendGyroZeros(flight_controller_t * flightController, uint8_t msg_len){
  Serial.println("SEND GYRO ZEROS");
  sensor_zero_msg_t szm;
  sprintf(szm.X, "%+08d", (long int)(flightController->mpu6500.gyroZero.axis.X * 100));
  sprintf(szm.Y, "%+08d", (long int)(flightController->mpu6500.gyroZero.axis.Y * 100));
  sprintf(szm.Z, "%+08d", (long int)(flightController->mpu6500.gyroZero.axis.Z * 100));
  sendMessage(SEND_GYRO_ZEROS, sizeof(szm), (char*)&szm);
}

void handleCommandSendAccZeros(flight_controller_t * flightController, uint8_t msg_len){
  Serial.println("SEND ACC ZEROS");
  sensor_zero_msg_t szm;
  sprintf(szm.X, "%+08d", (long int)(flightController->mpu6500.accZero.axis.X * 100));
  sprintf(szm.Y, "%+08d", (long int)(flightController->mpu6500.accZero.axis.Y * 100));
  sprintf(szm.Z, "%+08d", (long int)(flightController->mpu6500.accZero.axis.Z * 100));
  sendMessage(SEND_ACC_ZEROS, sizeof(szm), (char*)&szm);
}

void handleCommandSendMotorSpeeds(flight_controller_t * flightController, uint8_t msg_len){
  Serial.print("SEND MOTOR SPEEDS");
  motor_speed_msg_t msm;
  sprintf(msm.motor0, "%04d", static_cast<int>(flightController->motor[0]));
  sprintf(msm.motor1, "%04d", static_cast<int>(flightController->motor[1]));
  sprintf(msm.motor2, "%04d", static_cast<int>(flightController->motor[2]));
  sprintf(msm.motor3, "%04d", static_cast<int>(flightController->motor[3]));
  sendMessage(SEND_MOTOR_SPEEDS, sizeof(msm), (char*)&msm);
}

void handleCommandSendAngles(flight_controller_t * flightController, uint8_t msg_len){
  Serial.println("SEND ANGLES");
  angle_msg_t am;
  sprintf(am.roll, "%+06d", (int)(flightController->angle.axis.roll * 100));
  sprintf(am.pitch, "%+06d", (int)(flightController->angle.axis.pitch * 100));
  sprintf(am.yaw, "%+06u", (long int)(flightController->angle.axis.yaw * 100));
  sendMessage(SEND_ANGLES, sizeof(am), (char*)&am);
}

void handleCommandSetThrottle(flight_controller_t * flightController, uint8_t msg_len){
  Serial.println("SET THROTTLE");
  uint8_t new_throttle; 
  new_throttle = flightController->throttle;
  if (Serial.available() > 0)
    new_throttle = Serial.read();
  if(flightController->armed){
    flightController->setThrottle(new_throttle);
    if(!flightController->active)
      flightController->writeToMotors();
  }
  //flightController->active = true;
}

void fillBytes(char * data, uint8_t len){
  for(int i = 0; i < len && Serial.available() > 0; i++)
    data[i] = Serial.read();
}

bool getMessage(msg_t * msg, size_t len){
  if(Serial.available() > 0){
    (*msg).cmd = Serial.read(); 
  }
  else{
    return false;
  }
  if(Serial.available() > 0){
    (*msg).len = Serial.read();
  }
  else{
    return false;
  }

  return true;
}

bool headerFound(){
  const String commandHeader = String("$S>"); // Standard command header
  while(Serial.available() > 0){
    char recieved_byte = Serial.read();
    if(recieved_byte == commandHeader.charAt(0)){
      for(int i = 1; i < commandHeader.length(); i++){
        if(Serial.available() > 0)
          recieved_byte = Serial.read();
        else{
          return false;
        }
        if(recieved_byte != commandHeader.charAt(i)){
          clearSerialBuffer();
          return false;
        }
      }
      return true;          
    }
  }
  return false;
}

void clearSerialBuffer(){
  while(Serial.available() > 0){
    char recieved_byte = Serial.read();
  }
}

// with led facing you
// with mosfets facing you
// 9 is bottom left
// 10 is bottom right
// 6 is top left
// 5 is top right 

//Serial.print("  Roll: ");Serial.print(angle.axis.roll);
//Serial.print("  Pitch: ");Serial.print(angle.axis.pitch);
//Serial.print("  Yaw: ");Serial.println(angle.axis.yaw);

//Serial.println(dt,10);

/*Serial.print("Motor0: "); Serial.print(motor[0]);
Serial.print("  Motor1: "); Serial.print(motor[1]);
Serial.print("  Motor2: "); Serial.print(motor[2]);
Serial.print("  Motor3: "); Serial.println(motor[3]);*/

/*      if (Serial.available() > 0) {
        // read incoming serial data:
        char recieved_byte = Serial.read();
        // Type the next ASCII value from what you received:
        Serial.println(recieved_byte);
    
        if(recieved_byte == '1'){
          analogWrite(9, motorSpeed);
          analogWrite(10, motorSpeed);
          analogWrite(6, motorSpeed);
          analogWrite(3, motorSpeed);
    
          if(motorSpeed + 10 < 255)
            motorSpeed += 10;
        }
    
        else if(recieved_byte == '3'){
          analogWrite(9, 0);
          analogWrite(10, 0);
          analogWrite(6, 0);
          analogWrite(3, 0);
        }
    
        else if(recieved_byte == '5'){
          analogWrite(9, 255);
          analogWrite(10, 255);
          analogWrite(6, 255);
          analogWrite(3, 255);
        }
      }  
  */  


