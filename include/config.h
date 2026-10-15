// config.h - every pin and every tunable number lives here.
// Change values here, never inside the modules.
#pragma once
#include <Arduino.h>

namespace cfg {

// ============================ PIN MAP (Arduino Mega 2560) ====================
// L298N motor driver. Left motors (2x) on OUT1/OUT2, right motors (2x) on OUT3/OUT4.
constexpr uint8_t PIN_ENA = 5;    // D5  = OC3A (Timer3)  -> L298N ENA  (left  PWM)
constexpr uint8_t PIN_ENB = 6;    // D6  = OC4A (Timer4)  -> L298N ENB  (right PWM)
constexpr uint8_t PIN_IN1 = 22;   // PA0 -> L298N IN1  (left  direction)
constexpr uint8_t PIN_IN2 = 23;   // PA1 -> L298N IN2
constexpr uint8_t PIN_IN3 = 24;   // PA2 -> L298N IN3  (right direction)
constexpr uint8_t PIN_IN4 = 25;   // PA3 -> L298N IN4
// NOTE: motors.cpp drives IN1..IN4 through PORTA bits 0..3, so they must stay on D22..D25.

// Funduino 3-channel line tracker: index 0 = LEFT, 1 = CENTRE, 2 = RIGHT (looking from behind the robot)
constexpr uint8_t LINE_PINS[3] = {30, 31, 32};
constexpr bool LINE_BLACK_IS_HIGH = true;   // flip if the module outputs LOW on black

constexpr uint8_t PIN_START_BTN = 26;       // push button to GND (INPUT_PULLUP)
constexpr uint8_t PIN_LED = LED_BUILTIN;    // D13

// Reserved for Phase 1 / later: BLE module on Serial1 (TX1=D18, RX1=D19),
// encoders (if ever added) on D2/D3 (INT4/INT5).

// ============================ MOTORS =========================================
constexpr bool MOTOR_LEFT_INVERT  = false;  // flip if the left  side drives backwards
constexpr bool MOTOR_RIGHT_INVERT = false;  // flip if the right side drives backwards

// ============================ TIMING / SPEEDS ================================
constexpr uint16_t CONTROL_PERIOD_MS = 5;   // control loop = 200 Hz
constexpr int16_t  BASE_SPEED    = 130;     // PWM 0..255 while following a line
constexpr int16_t  MAX_SPEED     = 230;
constexpr int16_t  ADVANCE_SPEED = 120;     // open-loop speed while crossing a junction
constexpr int16_t  TURN_SPEED    = 120;     // pivot speed

// ============================ LINE FOLLOWING (PID) ===========================
// error = line position: -1 (line under LEFT) .. 0 (centre) .. +1 (line under RIGHT)
constexpr float PID_KP = 80.0f;
constexpr float PID_KI = 0.0f;
constexpr float PID_KD = 0.15f;             // per second
constexpr float PID_I_LIMIT = 50.0f;

// ============================ JUNCTION DETECTION =============================
// With only 3 sensors a small wobble can also turn LEFT or RIGHT black, so a side sensor alone is
// trusted only after it stays black for a while (a 3 cm branch keeps it black longer than a wobble).
constexpr uint16_t ALL3_CONFIRM_MS = 15;   // L+C+R all black (T, cross, circle): wobble cannot do this
constexpr uint16_t SIDE_CONFIRM_MS = 90;   // L or R alone black this long = branch/corner. TUNE with speed.
constexpr uint16_t SIDE_MIN_MS     = 60;   // black at least this long while crossing = that exit exists
constexpr uint16_t JUNCTION_ADVANCE_MS = 180;  // time from first detection until the AXLE is at the
                                               // junction centre. TUNE THIS (depends on where the
                                               // sensor is mounted relative to the wheels).
constexpr uint16_t LOST_LINE_MS        = 60;   // all-white this long while following = dead end
constexpr uint16_t LEAVE_START_CONFIRM_MS = 30;

// ============================ TURNING ========================================
constexpr uint16_t TURN_TIMEOUT_MS      = 2500;
constexpr uint8_t  TURN_EDGE_SAMPLES    = 2;    // consecutive centre-sensor samples to accept a line
constexpr uint16_t CIRCLE_UTURN_MIN_MS  = 450;  // U-turn inside a circle: spin at least this long
constexpr bool     UTURN_SPIN_LEFT      = true;

// ============================ MAZE MAP =======================================
constexpr uint8_t  MAX_NODES = 40;
// Loop closure uses a statistical gate: a node we arrive at is "the same node" as a known one if the
// position difference is within GATE_FLOOR + GATE_K * (accumulated position uncertainty).
// Uncertainty grows by POS_ERR_REL * (edge length) on every new edge we drive (no encoders -> time based).
constexpr float    POS_ERR_REL      = 0.08f;   // assumed relative error of a measured edge (8 %)
constexpr float    GATE_K           = 2.5f;
constexpr int32_t  GATE_FLOOR_TICKS = 150;
constexpr uint8_t  START_HEADING = 2;      // 0=N 1=E 2=S 3=W. In the given PDF the start line goes DOWN.

// ============================ DEBUG ==========================================
constexpr bool VERBOSE = true;             // print node events over Serial
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint16_t START_DELAY_MS = 1500;  // time to take your hand off the robot after pressing start

}  // namespace cfg
