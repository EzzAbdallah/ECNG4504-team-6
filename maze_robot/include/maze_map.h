// maze_map.h - internal representation of the maze (graph of nodes) + exploration planner.
//
// Directions are ABSOLUTE and fixed by the start heading: 0=N 1=E 2=S 3=W (x right, y down).
// Position of a node = dead-reckoned (x, y) in "ticks" = milliseconds of straight travel
// (there are no encoders, so time at constant speed is the distance measure).
#pragma once
#include <Arduino.h>
#include "config.h"

inline uint8_t dirLeft(uint8_t h)  { return (h + 3) & 3; }
inline uint8_t dirRight(uint8_t h) { return (h + 1) & 3; }
inline uint8_t dirBack(uint8_t h)  { return (h + 2) & 3; }
inline uint8_t dirBit(uint8_t d)   { return (uint8_t)(1u << d); }

constexpr int8_t NO_NODE = -1;
constexpr int8_t PLAN_DONE = -1;

struct Node {
  int32_t  x, y;       // dead-reckoned position (ticks)
  float    sx, sy;     // 1-sigma position uncertainty per axis (ticks)
  uint8_t  exits;      // bitmask of physical exits (bit d = exit towards direction d)
  bool     circle;     // start / end circle
  int8_t   link[4];    // neighbour node id per direction, -1 = not explored yet
  uint16_t len[4];     // measured edge length (ticks)
};

struct Arrival {
  int8_t id;           // node we are now standing on
  bool   created;      // a new node was added
  bool   known;        // we followed an already explored edge
  bool   loopClosed;   // an unexplored edge led to an already known node
  bool   overflow;     // node table full
};

class MazeMap {
 public:
  void reset(uint8_t startHeading);

  // Called every time the robot reaches a node.
  //   from    : node we left
  //   heading : absolute direction we travelled in
  //   ticks   : measured edge length
  //   exits   : exits observed at the new node (absolute bitmask)
  //   circle  : true if the new node is a start/end circle
  Arrival arrive(int8_t from, uint8_t heading, uint32_t ticks, uint8_t exits, bool circle);

  // Where to go next? Returns an absolute heading (0..3) or PLAN_DONE.
  // Priority: unexplored exit here (left, straight, right) -> nearest node with an unexplored
  // exit (BFS over the known graph) -> back to the start circle -> done.
  int8_t planNext(int8_t cur, uint8_t heading) const;

  uint8_t count() const { return n_; }
  const Node& node(uint8_t i) const { return nodes_[i]; }
  uint8_t loopClosures() const { return loops_; }

  void print(Print& out) const;      // table + ASCII drawing

 private:
  uint8_t unexplored(uint8_t i) const;
  int8_t bfsFirstHop(int8_t src, int8_t target) const;  // target<0 -> nearest node with unexplored exit

  Node    nodes_[cfg::MAX_NODES];
  uint8_t n_ = 0;
  uint8_t loops_ = 0;
};
