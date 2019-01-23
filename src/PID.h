#ifndef __pid_h__
#define __pid_h__

class pid_values_t{
  public:
    float Kp;
    float Ki;
    float Kd;
    float integrationLimit;
    float Fc;
  
};

class pid_control_t{
  public:
    void init(pid_values_t * pv);// get from eeprom
    void init(float kp, float ki, float kd, float integrationLimit, float Fc);
    double updatePID(double setPoint, double input, double dt);
    void resetRollPitchYaw();
    pid_values_t pidValues;

  private:
    float iTerm;
    float lastError;
  
};

#endif




