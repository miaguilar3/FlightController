#ifndef __gf_h__
#define __gf_h__
#include "Timer2.h"
#include "FlightController.h"
#define GRAPH_SAVE_VALUES 50

class graph_flight_t{

  Timer2 accelerate_timer;
  Timer2 deaccelerate_timer;
  Timer2 hover_timer;
  Timer2 cycle_timer;
  Timer2 graph_timer;

  bool accelerate_mode;
  bool hover_mode;
  uint16_t hover_time;
  bool deaccelerate_mode;
  int count;
  int throttle_step;
  
  uint16_t mv_index;
  uint16_t throttle;
  
public:

  uint16_t motor1_vals[GRAPH_SAVE_VALUES];
  uint16_t motor2_vals[GRAPH_SAVE_VALUES];
  uint16_t motor3_vals[GRAPH_SAVE_VALUES];
  uint16_t motor4_vals[GRAPH_SAVE_VALUES];

  graph_flight_t();
  void init();
  void gfh_update(flight_controller_t * flightController);
  void start(flight_controller_t * flightController);
  bool active;
};

#endif
