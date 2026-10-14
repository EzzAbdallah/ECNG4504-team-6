// motors.h - L298N driver, register-level (Timer3/Timer4 PWM + PORTA direction pins).
#pragma once
#include <Arduino.h>

class Motors {
 public:
  void begin();
  // Signed speeds: -255 (full reverse) .. +255 (full forward). Left/right = sides of the robot.
  void setSpeeds(int16_t left, int16_t right);
  void stop();    // active brake (both L298N inputs high)
  void coast();   // free-wheel (enable low)
 private:
  void setLeft(int16_t s);
  void setRight(int16_t s);
};
