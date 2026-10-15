// line_sensor.h - Funduino 3-channel digital line tracker (left, centre, right).
#pragma once
#include <Arduino.h>

struct LineReading {
  uint8_t mask;      // bit0 = LEFT, bit1 = CENTRE, bit2 = RIGHT (1 = sees black)
  uint8_t count;     // number of black sensors (0..3)
  float   position;  // -1 .. +1, weighted centre of the line (+ = line to the right)
};

constexpr uint8_t SENSOR_L = 0x01;
constexpr uint8_t SENSOR_C = 0x02;
constexpr uint8_t SENSOR_R = 0x04;

class LineSensor {
 public:
  static constexpr uint8_t kNum = 3;
  void begin();
  LineReading read();
 private:
  float lastPosition_ = 0.0f;
};
