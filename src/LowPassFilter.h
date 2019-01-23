#ifndef __low_pass_filter__
#define __low_pass_filter__

class lowPassFilter_t{

  private:
    
    
  public:
    lowPassFilter_t();
    lowPassFilter_t(float fc, float po);
    float applyLowPass(float input, double dt);
    float applyLowPass2(float input, double dt);
    float Fc;
    float prevOut; // gonna be a problem?
    
};

#endif





