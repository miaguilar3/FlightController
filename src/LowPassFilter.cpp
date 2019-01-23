#include "lowPassFilter.h"

#define M_PIf   3.14159265358979323846f

lowPassFilter_t::lowPassFilter_t(){

}

lowPassFilter_t::lowPassFilter_t(float fc, float po){
  //Fc = fc;
  //prevOut = po; // initialize something else?
}

float lowPassFilter_t::applyLowPass(float input, double dt){
  float tau = 1.0f / (2.0f * M_PIf * Fc);
  float alpha = dt / (tau + dt);

  // y(n) = y(n-1) + alpha*(u(n) - y(n-1))
  float output = prevOut + alpha * (input - prevOut);
  prevOut = output;
  return output;
}

float lowPassFilter_t::applyLowPass2(float input, double dt){

  return 0;  
}






