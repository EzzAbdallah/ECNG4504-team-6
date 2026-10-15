#include "line_sensor.h"
#include "config.h"

void LineSensor::begin() {
  for (uint8_t i = 0; i < 3; i++) pinMode(cfg::LINE_PINS[i], INPUT);
}

LineReading LineSensor::read() {
  static const int8_t kWeight[3] = {-1, 0, 1};
  LineReading r;
  r.mask = 0;
  r.count = 0;
  int16_t sum = 0;
  for (uint8_t i = 0; i < 3; i++) {
    const bool high = digitalRead(cfg::LINE_PINS[i]) == HIGH;
    const bool black = (high == cfg::LINE_BLACK_IS_HIGH);
    if (black) {
      r.mask |= (1 << i);
      r.count++;
      sum += kWeight[i];
    }
  }
  if (r.count > 0) {
    r.position = (float)sum / (float)r.count;
    lastPosition_ = r.position;
  } else {
    // line lost: saturate towards the side where we last saw it
    r.position = (lastPosition_ > 0) ? 1.0f : (lastPosition_ < 0 ? -1.0f : 0.0f);
  }
  return r;
}
