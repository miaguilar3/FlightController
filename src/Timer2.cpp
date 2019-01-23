#include "Arduino.h"
#include "Timer2.h"

void Timer2::init(){
  start_time = millis();
}

unsigned long Timer2::getElapsedTime(){
  unsigned long now = millis();
  unsigned long elapsed_time = now - start_time;
  return elapsed_time;
}

void Timer2::reset(){
  start_time = millis();
}



