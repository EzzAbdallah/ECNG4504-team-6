#include "motors.h"
#include "config.h"

// ---- direction bits on PORTA (D22..D25) ----
#define IN1 _BV(PA0)
#define IN2 _BV(PA1)
#define IN3 _BV(PA2)
#define IN4 _BV(PA3)

// ---- PWM helpers: Timer3 OC3A (D5, PE3) = left, Timer4 OC4A (D6, PH3) = right ----
// duty 0 disconnects the pin (a fast-PWM compare value of 0 would still give a 1-tick spike).
static inline void pwmLeft(uint8_t duty) {
  if (duty == 0) { TCCR3A &= ~_BV(COM3A1); PORTE &= ~_BV(PE3); }
  else           { OCR3A = duty; TCCR3A |= _BV(COM3A1); }
}
static inline void pwmRight(uint8_t duty) {
  if (duty == 0) { TCCR4A &= ~_BV(COM4A1); PORTH &= ~_BV(PH3); }
  else           { OCR4A = duty; TCCR4A |= _BV(COM4A1); }
}

void Motors::begin() {
  DDRA |= (IN1 | IN2 | IN3 | IN4);        // direction pins = outputs
  PORTA &= ~(IN1 | IN2 | IN3 | IN4);
  DDRE |= _BV(PE3);                       // D5 output
  DDRH |= _BV(PH3);                       // D6 output

  // Timer3 / Timer4: 8-bit fast PWM (WGM = 0101), non-inverting, clk/64 -> 16 MHz/64/256 = 976 Hz
  TCCR3A = _BV(WGM30);
  TCCR3B = _BV(WGM32) | _BV(CS31) | _BV(CS30);
  TCCR4A = _BV(WGM40);
  TCCR4B = _BV(WGM42) | _BV(CS41) | _BV(CS40);
  OCR3A = 0;
  OCR4A = 0;
  coast();
}

void Motors::setLeft(int16_t s) {
  if (cfg::MOTOR_LEFT_INVERT) s = -s;
  s = constrain(s, -255, 255);
  PORTA &= ~(IN1 | IN2);
  if (s > 0)      PORTA |= IN1;
  else if (s < 0) PORTA |= IN2;
  pwmLeft((uint8_t)abs(s));
}

void Motors::setRight(int16_t s) {
  if (cfg::MOTOR_RIGHT_INVERT) s = -s;
  s = constrain(s, -255, 255);
  PORTA &= ~(IN3 | IN4);
  if (s > 0)      PORTA |= IN3;
  else if (s < 0) PORTA |= IN4;
  pwmRight((uint8_t)abs(s));
}

void Motors::setSpeeds(int16_t left, int16_t right) {
  setLeft(left);
  setRight(right);
}

void Motors::stop() {                     // brake: IN1=IN2=1, IN3=IN4=1, EN high
  PORTA |= (IN1 | IN2 | IN3 | IN4);
  pwmLeft(255);
  pwmRight(255);
}

void Motors::coast() {
  PORTA &= ~(IN1 | IN2 | IN3 | IN4);
  pwmLeft(0);
  pwmRight(0);
}
