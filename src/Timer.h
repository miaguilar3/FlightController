#ifndef __timer_h__
#define __timer_h__

class Timer{
public:
  unsigned long getElapsedTime();
  void init();
  void reset();

private:
  unsigned long start_time;
};

#endif




