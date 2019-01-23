#include "PID.h"
#include "Arduino.h"

//void pid_t::init(pid_values_t * pv){
//  // get values from eeprom
//  pidValues = pv;
//}



void pid_control_t::init(float kp, float ki, float kd, float iLimit, float fc){
  pidValues.Kp = kp;
  pidValues.Ki = ki;
  pidValues.Kd = kd;
  pidValues.integrationLimit = iLimit;
  pidValues.Fc = fc;

  iTerm = 0;
  lastError = 0;
}

double pid_control_t::updatePID(double setPoint, double input, double dt){
  double error = setPoint - input;
  double pTerm = error * pidValues.Kp;

  iTerm += pidValues.Ki * error * dt;
  iTerm = constrain(iTerm, -pidValues.integrationLimit, pidValues.integrationLimit);
  
  //float deltaError = error - lastError;
  //lastError = error;

  return pTerm + iTerm;
}

void pid_control_t::resetRollPitchYaw(){
  lastError = 0.0f;
  // low_pass.prevOutput = 0.0f;
}






