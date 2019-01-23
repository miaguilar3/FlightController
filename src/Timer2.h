#ifndef __timer2_h__
#define __timer2_h__

class Timer2{
public:
  unsigned long getElapsedTime();
  void init();
  void reset();

private:
  unsigned long start_time;
};

#endif




