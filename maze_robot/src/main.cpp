// main.cpp - ECNG4504 maze robot, Phase 0 (training run).
// All the work happens in the modules; main only wires them together and runs the control loop.
#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "line_sensor.h"
#include "maze_map.h"
#include "robot_fsm.h"

static Motors motors;
static LineSensor lineSensor;
static MazeMap maze;
static RobotFSM robot(motors, lineSensor, maze);

void setup() {
  Serial.begin(cfg::SERIAL_BAUD);
  motors.begin();
  lineSensor.begin();
  robot.begin();
}

void loop() {
  static uint32_t nextTick = 0;
  const uint32_t now = millis();
  if ((int32_t)(now - nextTick) >= 0) {          // fixed 200 Hz control loop
    nextTick = now + cfg::CONTROL_PERIOD_MS;
    robot.update();
  }
}
