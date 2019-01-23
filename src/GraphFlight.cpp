#include"FlightController.h"
#include"GraphFlight.h"
#define LED_PIN A2

graph_flight_t::graph_flight_t(){

}

void graph_flight_t::init(){
  active = false;
  cycle_timer.init();
  accelerate_timer.init();
  deaccelerate_timer.init();
  hover_timer.init();
  graph_timer.init();
  accelerate_mode = false;
  hover_mode = false;
  deaccelerate_mode = false;
  count = 20;
  throttle_step = 30;
  throttle = 1640;
  hover_time = 7000;
  for(int i = 0; i < GRAPH_SAVE_VALUES; i++){
    motor1_vals[i] = 1000 + i;
    motor2_vals[i] = 1000 + i;
    motor3_vals[i] = 1000 + i;
    motor4_vals[i] = 1000 + i;
  }

  mv_index = 0;
}



void graph_flight_t::gfh_update(flight_controller_t * flightController){
  if(accelerate_mode && accelerate_timer.getElapsedTime() > 100){
     flightController->setThrottle2(flightController->throttle + throttle_step);
     count++;
     if(count >= 20){
        accelerate_mode = false;
        hover_mode = true;
        hover_timer.reset();
        //Serial.println("accelerate mode off");
        //Serial.println("hover mode on");
       }
       accelerate_timer.reset();
  }

  if(graph_timer.getElapsedTime() > 100){
    if(mv_index <  GRAPH_SAVE_VALUES){
      motor1_vals[mv_index] = flightController->motor[0];
      motor2_vals[mv_index] = flightController->motor[1];
      motor3_vals[mv_index] = flightController->motor[2];
      motor4_vals[mv_index] = flightController->motor[3];
      mv_index++;
    }
  }

  if(hover_mode && hover_timer.getElapsedTime() > hover_time){
    hover_mode = false;
    deaccelerate_mode = true;
    
    throttle_step = -throttle_step;
    deaccelerate_timer.reset();
    //Serial.println("hover mode off");
    //Serial.println("deaccelerate mode on");
  }

  if(deaccelerate_mode && deaccelerate_timer.getElapsedTime() > 50){
    flightController->setThrottle2(flightController->throttle + throttle_step);
     count--;
     if(count == 1){
      cycle_timer.reset();
     }
     if(count <= 0){
      deaccelerate_mode = false;
      Serial.println();
      Serial.println(cycle_timer.getElapsedTime());
      Serial.println("deaccelerate mode off");
      throttle_step = -throttle_step;
      flightController->active = false;
      flightController->setThrottle2(1000);
      active = false;
      flightController->writeToMotors();
      Serial.print("$S>?");
      Serial.println(char(0));
     }
     deaccelerate_timer.reset();
  }
}

void graph_flight_t::start(flight_controller_t * flightController){
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  flightController->active = true;
  flightController->setThrottle2(throttle);//1650 with teensy
  hover_mode = true;
  mv_index = 0;
  hover_timer.reset();
  graph_timer.reset();
  active = true;
}


  


