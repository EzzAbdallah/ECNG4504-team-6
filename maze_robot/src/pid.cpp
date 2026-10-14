#include "pid.h"

float PID::update(float error, float dt) {
  integral_ += error * dt * ki_;
  if (integral_ > iLimit_)  integral_ = iLimit_;
  if (integral_ < -iLimit_) integral_ = -iLimit_;

  float d = first_ ? 0.0f : (error - prevError_) / dt;
  first_ = false;
  prevError_ = error;
  dFilt_ += 0.3f * (d - dFilt_);            // low-pass: digital sensors make d very spiky

  return kp_ * error + integral_ + kd_ * dFilt_;
}
