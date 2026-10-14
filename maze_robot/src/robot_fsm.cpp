#include "robot_fsm.h"
#include "config.h"

RobotFSM::RobotFSM(Motors& m, LineSensor& s, MazeMap& map)
    : motors_(m), sensor_(s), map_(map),
      pid_(cfg::PID_KP, cfg::PID_KI, cfg::PID_KD, cfg::PID_I_LIMIT) {}

void RobotFSM::begin() {
  pinMode(cfg::PIN_START_BTN, INPUT_PULLUP);
  pinMode(cfg::PIN_LED, OUTPUT);
  Serial.println(F("\nECNG4504 maze robot - Phase 0 (training run)"));
  Serial.println(F("Commands: s=start  x=abort  d=sensor monitor  m=print map"));
  enter(State::IDLE);
}

const __FlashStringHelper* RobotFSM::name(State s) {
  switch (s) {
    case State::IDLE: return F("IDLE");
    case State::LEAVE_START: return F("LEAVE_START");
    case State::FOLLOW: return F("FOLLOW");
    case State::ADVANCE: return F("ADVANCE");
    case State::TURN: return F("TURN");
    case State::DONE: return F("DONE");
    default: return F("ERROR");
  }
}

void RobotFSM::enter(State s) {
  state_ = s;
  stateSince_ = millis();
  if (s == State::IDLE || s == State::DONE || s == State::ERROR) {
    motors_.stop();
    delay(60);
    motors_.coast();
  }
  if (s == State::DONE) {
    digitalWrite(cfg::PIN_LED, HIGH);
    Serial.println(F("Training complete: whole maze explored, back at the start."));
    map_.print(Serial);
  }
}

void RobotFSM::fail(const __FlashStringHelper* why) {
  Serial.print(F("ERROR: ")); Serial.println(why);
  enter(State::ERROR);
}

void RobotFSM::update() {
  pollSerial();
  const LineReading r = sensor_.read();
  switch (state_) {
    case State::IDLE:        doIdle(r); break;
    case State::LEAVE_START: doLeaveStart(r); break;
    case State::FOLLOW:      doFollow(r); break;
    case State::ADVANCE:     doAdvance(r); break;
    case State::TURN:        doTurn(r); break;
    case State::DONE:        break;
    case State::ERROR:       doError(); break;
  }
}

// ---------------------------------------------------------------- serial / idle
void RobotFSM::pollSerial() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == 'x') {
      Serial.println(F("ABORT"));
      armed_ = false;
      enter(State::IDLE);
    } else if (c == 's' && (state_ == State::IDLE || state_ == State::DONE || state_ == State::ERROR)) {
      armed_ = true;
      startAt_ = millis() + cfg::START_DELAY_MS;
      state_ = State::IDLE;
      Serial.println(F("Starting..."));
    } else if (c == 'd') {
      diag_ = !diag_;
    } else if (c == 'm') {
      map_.print(Serial);
    }
  }
}

void RobotFSM::printLine(const LineReading& r) {
  for (int8_t i = 0; i < 5; i++) Serial.print((r.mask >> i) & 1 ? '#' : '.');
  Serial.print(F("  count=")); Serial.print(r.count);
  Serial.print(F("  pos=")); Serial.println(r.position, 1);
}

void RobotFSM::doIdle(const LineReading& r) {
  digitalWrite(cfg::PIN_LED, (millis() / 500) & 1);
  if (diag_ && millis() - lastDiagMs_ > 100) {   // sensor monitor: check wiring / polarity / pitch
    lastDiagMs_ = millis();
    printLine(r);
  }
  // button (active low), ~50 ms debounce
  if (digitalRead(cfg::PIN_START_BTN) == LOW) {
    if (btnCount_ < 255) btnCount_++;
    if (btnCount_ == 10 && !armed_) { armed_ = true; startAt_ = millis() + cfg::START_DELAY_MS; }
  } else {
    btnCount_ = 0;
  }
  if (armed_ && (int32_t)(millis() - startAt_) >= 0) {
    armed_ = false;
    map_.reset(cfg::START_HEADING);
    heading_ = cfg::START_HEADING;
    curNode_ = 0;
    firstSegment_ = true;
    leaveActive_ = false;
    digitalWrite(cfg::PIN_LED, LOW);
    enter(State::LEAVE_START);
  }
}

// ---------------------------------------------------------------- leave start circle
void RobotFSM::doLeaveStart(const LineReading& r) {
  const uint32_t now = millis();
  motors_.setSpeeds(cfg::BASE_SPEED, cfg::BASE_SPEED);
  if (r.count < cfg::JUNCTION_MIN_BLACK) {            // sensor bar left the black circle
    if (!leaveActive_) { leaveActive_ = true; leaveStart_ = now; }
    if (now - leaveStart_ >= cfg::LEAVE_START_CONFIRM_MS) {
      startFollowing();
      segStartMs_ = leaveStart_;
    }
  } else {
    leaveActive_ = false;
  }
  if (now - stateSince_ > 4000) fail(F("never left the start circle"));
}

void RobotFSM::startFollowing() {
  pid_.reset();
  candActive_ = false;
  lostActive_ = false;
  segStartMs_ = millis();
  enter(State::FOLLOW);
}

// ---------------------------------------------------------------- line following
void RobotFSM::doFollow(const LineReading& r) {
  const uint32_t now = millis();
  const int16_t base = cfg::BASE_SPEED;

  // junction / corner / circle: most sensors black
  if (r.count >= cfg::JUNCTION_MIN_BLACK) {
    if (!candActive_) { candActive_ = true; candStart_ = now; candMask_ = 0; }
    candMask_ |= r.mask;
    motors_.setSpeeds(base, base);                    // go straight while we confirm
    lostActive_ = false;
    if (now - candStart_ >= cfg::JUNCTION_CONFIRM_MS) {
      leftSeen_ = candMask_ & SENSOR_S0;
      rightSeen_ = candMask_ & SENSOR_S4;
      enter(State::ADVANCE);
    }
    return;
  }
  candActive_ = false;

  // line lost: dead end if it stays lost
  if (r.count == 0) {
    if (!lostActive_) { lostActive_ = true; lostStart_ = now; }
    motors_.setSpeeds(base, base);
    if (now - lostStart_ >= cfg::LOST_LINE_MS) {
      motors_.stop();
      uint32_t ticks = lostStart_ - segStartMs_ + (firstSegment_ ? 0 : cfg::JUNCTION_ADVANCE_MS);
      handleNode(dirBit(dirBack(heading_)), false, ticks);
    }
    return;
  }
  lostActive_ = false;

  // normal PID line following
  const float dt = cfg::CONTROL_PERIOD_MS / 1000.0f;
  const float corr = pid_.update(r.position, dt);
  motors_.setSpeeds(constrain((int)(base + corr), -cfg::MAX_SPEED, cfg::MAX_SPEED),
                    constrain((int)(base - corr), -cfg::MAX_SPEED, cfg::MAX_SPEED));
}

// ---------------------------------------------------------------- cross the junction
void RobotFSM::doAdvance(const LineReading& r) {
  const uint32_t now = millis();
  motors_.setSpeeds(cfg::ADVANCE_SPEED, cfg::ADVANCE_SPEED);
  if (r.mask & SENSOR_S0) leftSeen_ = true;
  if (r.mask & SENSOR_S4) rightSeen_ = true;
  if (now - candStart_ < cfg::JUNCTION_ADVANCE_MS) return;

  motors_.stop();
  // The sensor bar is now just past the junction centre:
  const bool straight = (r.mask & SENSOR_CENTER_MASK) != 0;   // line continues ahead
  const bool circle = (r.count >= 5);                         // still all black = big circle
  const uint8_t h = heading_;
  uint8_t exits = dirBit(dirBack(h));
  if (!circle) {
    if (straight)  exits |= dirBit(h);
    if (leftSeen_) exits |= dirBit(dirLeft(h));
    if (rightSeen_) exits |= dirBit(dirRight(h));
  }
  const uint32_t ticks = candStart_ - segStartMs_ + (firstSegment_ ? 0 : cfg::JUNCTION_ADVANCE_MS);
  handleNode(exits, circle, ticks);
}

// ---------------------------------------------------------------- node reached: update map, choose
void RobotFSM::handleNode(uint8_t exitsAbs, bool circle, uint32_t ticks) {
  const Arrival a = map_.arrive(curNode_, heading_, ticks, exitsAbs, circle);
  if (a.overflow) { fail(F("node table full (map not closing - tune POS_ERR_REL / GATE_K)")); return; }
  curNode_ = a.id;
  firstSegment_ = false;

  if (cfg::VERBOSE) {
    const Node& n = map_.node(a.id);
    Serial.print(F("node ")); Serial.print(a.id);
    Serial.print(a.created ? F(" NEW") : (a.loopClosed ? F(" LOOP-CLOSED") : F(" known")));
    Serial.print(circle ? F(" circle") : F(""));
    Serial.print(F(" exits=")); Serial.print(exitsAbs, BIN);
    Serial.print(F(" pos=(")); Serial.print((long)n.x); Serial.print(',');
    Serial.print((long)n.y); Serial.print(F(") ticks=")); Serial.println(ticks);
  }

  const int8_t next = map_.planNext(curNode_, heading_);
  if (next < 0) { enter(State::DONE); return; }
  beginTurn((uint8_t)next);
}

// ---------------------------------------------------------------- turning
void RobotFSM::beginTurn(uint8_t target) {
  const uint8_t rel = (target - heading_ + 4) & 3;     // 0 straight, 1 right, 2 u-turn, 3 left
  targetHeading_ = target;
  if (rel == 0) { startFollowing(); return; }

  spinLeft_ = (rel == 3) || (rel == 2 && cfg::UTURN_SPIN_LEFT);
  const Node& n = map_.node(curNode_);
  // While pivoting, the centre sensor sees a line for every exit we sweep over, so
  // we know how many "line appearances" to wait for.
  edgesNeeded_ = 0;
  uint8_t d = heading_;
  for (uint8_t k = 0; k < (rel == 2 ? 2 : 1); k++) {
    d = spinLeft_ ? dirLeft(d) : dirRight(d);
    if (n.exits & dirBit(d)) edgesNeeded_++;
  }
  if (edgesNeeded_ == 0) edgesNeeded_ = 1;             // safety
  minSpinMs_ = (rel == 2 && n.circle) ? cfg::CIRCLE_UTURN_MIN_MS : 0;
  edgesSeen_ = 0;
  s2Run_ = 0;
  whiteSeen_ = false;                                   // set on first white sample
  turnStart_ = millis();
  enter(State::TURN);
}

void RobotFSM::doTurn(const LineReading& r) {
  const uint32_t now = millis();
  if (spinLeft_) motors_.setSpeeds(-cfg::TURN_SPEED, cfg::TURN_SPEED);
  else           motors_.setSpeeds(cfg::TURN_SPEED, -cfg::TURN_SPEED);

  const bool s2 = r.mask & SENSOR_S2;
  if (s2) {
    if (s2Run_ < 255) s2Run_++;
  } else {
    s2Run_ = 0;
    whiteSeen_ = true;
  }
  if (s2Run_ == cfg::TURN_EDGE_SAMPLES && whiteSeen_ && (now - turnStart_) >= minSpinMs_) {
    edgesSeen_++;
    whiteSeen_ = false;
    if (edgesSeen_ >= edgesNeeded_) {
      heading_ = targetHeading_;
      startFollowing();
      return;
    }
  }
  if (now - turnStart_ > cfg::TURN_TIMEOUT_MS) fail(F("turn timed out (line not found)"));
}

void RobotFSM::doError() {
  digitalWrite(cfg::PIN_LED, (millis() / 150) & 1);
}
