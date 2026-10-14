// line_sensor.h - 5-channel digital line sensor.
#pragma once
#include <Arduino.h>

struct LineReading {
  uint8_t mask;      // bit i = sensor i sees black (bit0 = leftmost, bit4 = rightmost)
  uint8_t count;     // number of black sensors
  float   position;  // -2..+2, weighted centre of the line (+ = line to the right)
};

constexpr uint8_t SENSOR_S0 = 0x01;   // outer left
constexpr uint8_t SENSOR_S2 = 0x04;   // centre
constexpr uint8_t SENSOR_S4 = 0x10;   // outer right
constexpr uint8_t SENSOR_CENTER_MASK = 0x0E;  // S1..S3

class LineSensor {
 public:
  void begin();
  LineReading read();
 private:
  float lastPosition_ = 0.0f;
};
