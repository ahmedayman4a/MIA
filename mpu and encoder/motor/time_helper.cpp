#include <sys/_stdint.h>
#include "wiring_time.h"
#include <Arduino.h>
#include "time_helper.h"


TimeHelper::TimeHelper() {
  TimeHelper::previous_time = 0;
}

double TimeHelper::get_dt() {
  
  // if this is the first reading there is no dt
  if (TimeHelper::previous_time == 0) {
    previous_time = millis();
    return 0;
  }

  uint32_t current_time = millis();
  double dt = (current_time*1.0 - TimeHelper::previous_time*1.0)/1000.0;

  TimeHelper::previous_time = current_time;
  return dt;
}