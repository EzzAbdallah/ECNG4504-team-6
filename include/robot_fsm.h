// robot_fsm.h - top-level finite state machine for the training run (Phase 0).
//
//   IDLE --start--> LEAVE_START --> FOLLOW <--------------------------+
//                                     | junction/corner/circle        |
//                                     v                               |
//                                  ADVANCE --classify, update map--> TURN
//                                     |  (dead end: handled directly in FOLLOW -> TURN)
//                                     +--> DONE (everything explored, back at start)
//   any state --error--> ERROR        'x' over Serial = abort -> IDLE
#pragma once
#include <Arduino.h>
#include "motors.h"
#include "line_sensor.h"
#include "pid.h"
#include "maze_map.h"

enum class State : uint8_t { IDLE, LEAVE_START, FOLLOW, ADVANCE, TURN, DONE, ERROR };

class RobotFSM {
 public:
  RobotFSM(Motors& m, LineSensor& s, MazeMap& map);
  void begin();
  void update();   // call every cfg::CONTROL_PERIOD_MS

 private:
  // state handlers
  void doIdle(const LineReading& r);
  void doLeaveStart(const LineReading& r);
  void doFollow(const LineReading& r);
  void doAdvance(const LineReading& r);
  void doTurn(const LineReading& r);
  void doError();

  // helpers
  void enter(State s);
  void startFollowing();
  void handleNode(uint8_t exitsAbs, bool circle, uint32_t ticks);
  void beginTurn(uint8_t target);
  void fail(const __FlashStringHelper* why);
  void pollSerial();
  void printLine(const LineReading& r);
  static const __FlashStringHelper* name(State s);

  Motors& motors_;
  LineSensor& sensor_;
  MazeMap& map_;
  PID pid_;

  State state_ = State::IDLE;
  uint32_t stateSince_ = 0;

  // run state
  uint8_t heading_ = 0;
  int8_t  curNode_ = 0;
  uint32_t segStartMs_ = 0;
  bool firstSegment_ = true;

  // idle / start
  bool armed_ = false;
  uint32_t startAt_ = 0;
  bool diag_ = false;
  uint32_t lastDiagMs_ = 0;
  bool leaveActive_ = false;
  uint32_t leaveStart_ = 0;
  uint8_t btnCount_ = 0;

  // follow
  uint32_t lRun_ = 0, rRun_ = 0, allRun_ = 0;   // how long (ms) each condition has been black without a break
  uint32_t candStart_ = 0;                      // when the junction pattern first appeared
  bool lostActive_ = false;
  uint32_t lostStart_ = 0;

  // advance
  uint32_t lBlackMs_ = 0, rBlackMs_ = 0;        // black time of the side sensors while crossing

  // turn
  uint8_t targetHeading_ = 0;
  bool spinLeft_ = true;
  uint8_t edgesNeeded_ = 0, edgesSeen_ = 0, s2Run_ = 0;
  bool whiteSeen_ = false;
  uint16_t minSpinMs_ = 0;
  uint32_t turnStart_ = 0;
};
