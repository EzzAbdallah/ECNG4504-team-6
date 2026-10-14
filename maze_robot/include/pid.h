// pid.h - small PID controller with filtered derivative and anti-windup.
#pragma once
#include <Arduino.h>

class PID {
 public:
  PID(float kp, float ki, float kd, float iLimit)
      : kp_(kp), ki_(ki), kd_(kd), iLimit_(iLimit) {}
  void reset() { integral_ = 0; prevError_ = 0; dFilt_ = 0; first_ = true; }
  float update(float error, float dt);
 private:
  float kp_, ki_, kd_, iLimit_;
  float integral_ = 0, prevError_ = 0, dFilt_ = 0;
  bool first_ = true;
};
