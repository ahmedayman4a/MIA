#include "motor_pid.h"
#include <Arduino.h>

MotorPID::MotorPID(int target_rpm, float Kp, float Ki, float Kd, int rpm_clamp_limit) {
  MotorPID::target_rpm = target_rpm;
  MotorPID::Kp = Kp;
  MotorPID::Ki = Ki;
  MotorPID::Kd = Kd;
  MotorPID::rpm_clamp_limit = rpm_clamp_limit;
  MotorPID::last_rpm = 0;
  MotorPID::eIntegral = 0;
}

MotorPID::MotorPID(int target_rpm, float Kp, float Ki, float Kd) {
  MotorPID(target_rpm, Kp, Ki, Kd, 0);
}

double MotorPID::clamp_saturation(double controlled_rpm) {
  // the rpm at which the integrator is turned off to prevent integral wind-up
  // for explanation, refer to : https://www.youtube.com/watch?v=NVLXCwc8HzM
  if (MotorPID::rpm_clamp_limit > 0 && abs(controlled_rpm) > MotorPID::rpm_clamp_limit) {
    return MotorPID::rpm_clamp_limit;
  } else {
    return controlled_rpm;
  }
}


double MotorPID::get_controlled_rpm(double rpm, double dt) {

  double error = MotorPID::target_rpm - rpm;
  double newIntegral = MotorPID::eIntegral + error;
  double eDerivative = -(rpm - MotorPID::last_rpm) / dt;
  double controlled_rpm = MotorPID::Kp * error + MotorPID::Ki * newIntegral + MotorPID::Kd * eDerivative;

  // double clamped_rpm = MotorPID::clamp_saturation(controlled_rpm);

  // bool stop_integral_update = false;

  // // if rpm has been clamped and integrator is still trying to go past saturation, then turn off integrator
  // if (clamped_rpm != controlled_rpm && error * controlled_rpm > 0) {
  //   stop_integral_update = true;
  // }


  // if integral isnt turned off update eIntegral
  //if (!stop_integral_update) {
  MotorPID::eIntegral = newIntegral;
  //}
  MotorPID::last_rpm = rpm;
  return controlled_rpm;
}