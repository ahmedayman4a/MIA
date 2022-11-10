#include <sys/types.h>
#include "encoder.h"
#include <ros.h>
#define MINUTE 60.0

Encoder::Encoder(double resolution) {
  Encoder::resolution = resolution;
}

double Encoder::count_to_rpm(u_int32_t count, double dt, ros::NodeHandle& nh) {
  u_int32_t dcount = count - Encoder::previous_count;
  nh.loginfo("dcount");
  nh.loginfo(String(dcount).c_str());
  double rotations = dcount / Encoder::resolution;
  nh.loginfo("rotations");
  nh.loginfo(String(rotations).c_str());
  Encoder::previous_count = count;
  double rps = rotations / dt;
  nh.loginfo("rps");
  nh.loginfo(String(rps).c_str());
  double rpm = rps * MINUTE;
  nh.loginfo("rpm");
  nh.loginfo(String(rpm).c_str());
  return rpm;
}