#include "Arduino.h"
#include "Timer.h"

void Timer::init(){
  start_time = micros();
}

unsigned long Timer::getElapsedTime(){
  unsigned long now = micros();
  unsigned long elapsed_time = now - start_time;
  return elapsed_time;
}

void Timer::reset(){
  start_time = micros();
}



