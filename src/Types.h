#ifndef __types_h__
#define __types_h__
#include "Arduino.h"

typedef union {
    struct {
        int16_t X, Y, Z;
    } __attribute__((packed)) axis;
    int16_t data[3]; // was int16_t
} sensorRaw_t;

typedef union {
    struct {
        float X, Y, Z;
    } __attribute__((packed)) axis;
    float data[3];
} sensor_t;

typedef union {
    struct {
        float roll, pitch, yaw;
    } __attribute__((packed)) axis;
    float data[3];
} angle_t;

typedef struct {
    uint8_t cmd;
    uint8_t len;
} __attribute__((packed)) msg_t;

typedef struct{
  char motor0[5];
  char motor1[5];
  char motor2[5];
  char motor3[5];
} __attribute__((packed)) motor_speed_msg_t;

typedef struct{
  char roll[7];
  char pitch[7];
  char yaw[7];
} __attribute__((packed)) angle_msg_t;

typedef struct{
  char X[9];
  char Y[9];
  char Z[9];
} __attribute__((packed)) sensor_zero_msg_t;

typedef struct{
  char P[7];
  char I[7];
  char D[7];
} __attribute__((packed)) pid_constants_msg_t;

typedef struct{
  char P[9];
  char I[9];
  char D[9];
} __attribute__((packed)) pid_terms_msg_t;


#endif




