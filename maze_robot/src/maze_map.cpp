#include "maze_map.h"
#include <math.h>

static const int8_t DX[4] = {0, 1, 0, -1};
static const int8_t DY[4] = {-1, 0, 1, 0};
static const char DIRCH[4] = {'N', 'E', 'S', 'W'};

static uint8_t popcount4(uint8_t v) {
  return (v & 1) + ((v >> 1) & 1) + ((v >> 2) & 1) + ((v >> 3) & 1);
}

void MazeMap::reset(uint8_t startHeading) {
  n_ = 0;
  loops_ = 0;
  Node& s = nodes_[n_++];
  s.x = 0; s.y = 0; s.sx = 0; s.sy = 0;
  s.exits = dirBit(startHeading);   // the start circle has exactly one line leaving it
  s.circle = true;
  for (uint8_t d = 0; d < 4; d++) { s.link[d] = NO_NODE; s.len[d] = 0; }
}

uint8_t MazeMap::unexplored(uint8_t i) const {
  uint8_t m = 0;
  for (uint8_t d = 0; d < 4; d++)
    if ((nodes_[i].exits & dirBit(d)) && nodes_[i].link[d] < 0) m |= dirBit(d);
  return m;
}

Arrival MazeMap::arrive(int8_t from, uint8_t h, uint32_t ticks, uint8_t exits, bool circle) {
  Arrival a = {NO_NODE, false, false, false, false};
  Node& p = nodes_[from];
  const uint8_t back = dirBack(h);

  // 1) Edge already explored: topology tells us where we are, and we re-anchor (drift reset).
  if (p.link[h] >= 0) {
    a.id = p.link[h];
    a.known = true;
    return a;
  }

  // 2) Unexplored edge. Dead-reckon the new position along the (axis-aligned) line.
  const int32_t nx = p.x + (int32_t)DX[h] * (int32_t)ticks;
  const int32_t ny = p.y + (int32_t)DY[h] * (int32_t)ticks;
  const uint16_t len = ticks > 65535UL ? 65535 : (uint16_t)ticks;

  // 3) Loop closure: is this an already known node? It must be of the same kind with the same
  //    exits, its exit that points back to us must still be unexplored, and the position must
  //    agree within the uncertainty gate. Along the travel axis the uncertainty grows by
  //    POS_ERR_REL * ticks; the other axis is inherited from the node we left.
  const bool horiz = (h == 1 || h == 3);
  const float err = cfg::POS_ERR_REL * (float)ticks;
  const float nsx = horiz ? sqrtf(p.sx * p.sx + err * err) : p.sx;
  const float nsy = horiz ? p.sy : sqrtf(p.sy * p.sy + err * err);
  int8_t best = NO_NODE;
  float bestScore = 1e9f;
  for (uint8_t i = 0; i < n_; i++) {
    if ((int8_t)i == from) continue;
    const Node& c = nodes_[i];
    if (c.circle != circle || c.exits != exits || c.link[back] != NO_NODE) continue;
    const float gx = cfg::GATE_FLOOR_TICKS + cfg::GATE_K * sqrtf(nsx * nsx + c.sx * c.sx);
    const float gy = cfg::GATE_FLOOR_TICKS + cfg::GATE_K * sqrtf(nsy * nsy + c.sy * c.sy);
    const float dx = (float)abs(c.x - nx), dy = (float)abs(c.y - ny);
    if (dx > gx || dy > gy) continue;
    const float score = dx / gx + dy / gy;
    if (score < bestScore) { bestScore = score; best = i; }
  }
  if (best >= 0) {
    p.link[h] = best;       p.len[h] = len;
    nodes_[best].link[back] = from; nodes_[best].len[back] = len;
    loops_++;
    a.id = best;
    a.loopClosed = true;
    return a;
  }

  // 4) New node.
  if (n_ >= cfg::MAX_NODES) { a.overflow = true; return a; }
  const int8_t id = n_++;
  Node& c = nodes_[id];
  c.x = nx; c.y = ny; c.sx = nsx; c.sy = nsy;
  c.exits = exits;
  c.circle = circle;
  for (uint8_t d = 0; d < 4; d++) { c.link[d] = NO_NODE; c.len[d] = 0; }
  c.link[back] = from; c.len[back] = len;
  p.link[h] = id;      p.len[h] = len;
  a.id = id;
  a.created = true;
  return a;
}

// Breadth-first search over the known graph. Returns the first direction to take from src, or -1.
int8_t MazeMap::bfsFirstHop(int8_t src, int8_t target) const {
  int8_t firstHop[cfg::MAX_NODES];
  int8_t queue[cfg::MAX_NODES];
  bool seen[cfg::MAX_NODES];
  for (uint8_t i = 0; i < n_; i++) { seen[i] = false; firstHop[i] = -1; }
  uint8_t qh = 0, qt = 0;
  queue[qt++] = src;
  seen[src] = true;
  while (qh < qt) {
    const int8_t u = queue[qh++];
    if (u != src) {
      const bool hit = (target >= 0) ? (u == target) : (unexplored(u) != 0);
      if (hit) return firstHop[u];
    }
    for (uint8_t d = 0; d < 4; d++) {
      const int8_t v = nodes_[u].link[d];
      if (v >= 0 && !seen[v]) {
        seen[v] = true;
        firstHop[v] = (u == src) ? (int8_t)d : firstHop[u];
        queue[qt++] = v;
      }
    }
  }
  return -1;
}

int8_t MazeMap::planNext(int8_t cur, uint8_t h) const {
  // 1) unexplored exit right here: prefer left, straight, right (left-hand preference)
  const uint8_t order[3] = {dirLeft(h), h, dirRight(h)};
  const uint8_t u = unexplored(cur);
  for (uint8_t k = 0; k < 3; k++)
    if (u & dirBit(order[k])) return order[k];
  if (u & dirBit(dirBack(h))) return dirBack(h);

  // 2) nearest node that still has an unexplored exit
  int8_t hop = bfsFirstHop(cur, -1);
  if (hop >= 0) return hop;

  // 3) everything explored: go back to the start circle (node 0)
  if (cur == 0) return PLAN_DONE;
  hop = bfsFirstHop(cur, 0);
  return hop;   // -1 (= PLAN_DONE) if unreachable
}

// ----------------------------------------------------------------------------
// Printing: node table + ASCII drawing (y grows downwards, like the track PDF)
// ----------------------------------------------------------------------------
static char symbolOf(const Node& n, uint8_t i) {
  if (i == 0) return 'S';
  if (n.circle) return 'E';
  switch (popcount4(n.exits)) {
    case 1: return 'D';   // dead end
    case 2: return 'o';   // corner
    default: return '+';  // junction
  }
}

void MazeMap::print(Print& out) const {
  out.println(F("\n===== MAZE MAP ====="));
  out.println(F("id sym   x      y     exits  links (dir>node:len)"));
  uint8_t deadEnds = 0, junctions = 0, corners = 0, edges2 = 0;
  for (uint8_t i = 0; i < n_; i++) {
    const Node& n = nodes_[i];
    const char s = symbolOf(n, i);
    if (s == 'D') deadEnds++;
    else if (s == '+') junctions++;
    else if (s == 'o') corners++;
    out.print(i); out.print(F("  ")); out.print(s); out.print(F("  "));
    out.print((long)n.x); out.print(F("  ")); out.print((long)n.y); out.print(F("  "));
    for (uint8_t d = 0; d < 4; d++) if (n.exits & dirBit(d)) out.print(DIRCH[d]);
    out.print(F("  "));
    for (uint8_t d = 0; d < 4; d++) {
      if (n.link[d] >= 0) {
        edges2++;
        out.print(DIRCH[d]); out.print('>'); out.print(n.link[d]);
        out.print('('); out.print(n.len[d]); out.print(F(") "));
      } else if (n.exits & dirBit(d)) {
        out.print(DIRCH[d]); out.print(F(">? "));
      }
    }
    out.println();
  }
  out.print(F("nodes=")); out.print(n_);
  out.print(F(" junctions=")); out.print(junctions);
  out.print(F(" corners=")); out.print(corners);
  out.print(F(" dead_ends=")); out.print(deadEnds);
  out.print(F(" edges=")); out.print(edges2 / 2);
  out.print(F(" loop_closures=")); out.println(loops_);

  // ASCII drawing
  const uint8_t COLS = 61, ROWS = 31;
  static char cv[ROWS][COLS + 1];
  for (uint8_t r = 0; r < ROWS; r++) { for (uint8_t c = 0; c < COLS; c++) cv[r][c] = ' '; cv[r][COLS] = 0; }
  int32_t minX = nodes_[0].x, maxX = minX, minY = nodes_[0].y, maxY = minY;
  for (uint8_t i = 1; i < n_; i++) {
    minX = min(minX, nodes_[i].x); maxX = max(maxX, nodes_[i].x);
    minY = min(minY, nodes_[i].y); maxY = max(maxY, nodes_[i].y);
  }
  const int32_t rx = max((int32_t)1, maxX - minX), ry = max((int32_t)1, maxY - minY);
  uint8_t col[cfg::MAX_NODES], row[cfg::MAX_NODES];
  for (uint8_t i = 0; i < n_; i++) {
    col[i] = (uint8_t)(((nodes_[i].x - minX) * (COLS - 1)) / rx);
    row[i] = (uint8_t)(((nodes_[i].y - minY) * (ROWS - 1)) / ry);
  }
  for (uint8_t i = 0; i < n_; i++) {
    const int8_t e = nodes_[i].link[1], s = nodes_[i].link[2];
    if (e >= 0) for (uint8_t c = col[i] + 1; c < col[e]; c++) cv[row[i]][c] = '-';
    if (s >= 0) for (uint8_t r = row[i] + 1; r < row[s]; r++) cv[r][col[i]] = '|';
  }
  for (uint8_t i = 0; i < n_; i++) cv[row[i]][col[i]] = symbolOf(nodes_[i], i);
  out.println(F("\nS=start  E=end circle  +=junction  o=corner  D=dead end"));
  for (uint8_t r = 0; r < ROWS; r++) out.println(cv[r]);
  out.println(F("===== END MAP ====="));
}
