# ECNG4504 Maze Robot - Phase 0 (training run)

Arduino Mega 2560 + L298N + Funduino 3-channel line tracker, PlatformIO, C++ only.
The robot follows the line (PID), stops at every node, records it in a graph, explores the
**whole** maze (loops and dead ends included) and returns to the start circle. No shortest path
and no BLE yet (Phase 1).

## Code layout (one module = one .h + one .cpp)
| Module | Job |
|---|---|
| `config.h` | **All pins and all tunable numbers.** Edit here only. |
| `motors` | L298N driver. Register-level: Timer3/Timer4 fast PWM on D5/D6, direction on PORTA (D22-D25). |
| `line_sensor` | Reads the 3 sensors (left, centre, right) -> bit mask, count, weighted position (-1..+1). |
| `pid` | PID with filtered derivative and anti-windup. |
| `maze_map` | Graph of nodes, dead reckoning, loop closure, BFS planner, map printer. |
| `robot_fsm` | The state machine that ties everything together. |
| `main.cpp` | Creates the objects and runs a fixed 200 Hz control loop. |

## Pin map (Arduino Mega 2560)
| Signal | Pin | Note |
|---|---|---|
| L298N ENA (left PWM) | D5 | OC3A, Timer3 |
| L298N ENB (right PWM) | D6 | OC4A, Timer4 |
| L298N IN1 / IN2 (left dir) | D22 / D23 | PA0 / PA1 |
| L298N IN3 / IN4 (right dir) | D24 / D25 | PA2 / PA3 |
| Start button (to GND) | D26 | INPUT_PULLUP |
| Line sensor LEFT / CENTRE / RIGHT | D30 / D31 / D32 | sensor VCC=5V, GND common (check the header labels on the back of the board) |
| Status LED | D13 | built in |
| Reserved: BLE (Phase 1) | D18/D19 | Serial1 |
| Reserved: encoders (if added) | D2/D3 | INT4/INT5 |

Left side = 2 motors in parallel on OUT1/OUT2. Right side = 2 motors in parallel on OUT3/OUT4.
Battery GND, L298N GND, Arduino GND and sensor GND must all be joined.
If a side spins the wrong way, flip `MOTOR_LEFT_INVERT` / `MOTOR_RIGHT_INVERT`.

## FSM
```
IDLE --button / 's'--> LEAVE_START --> FOLLOW --junction--> ADVANCE --> (update map, plan) --> TURN --> FOLLOW
                                         |  line lost 60 ms = dead end ------> (update map, plan) --> TURN
                                         '--> DONE (all explored, back at start: map printed)
any state --> ERROR (turn timeout, node table full)        'x' over Serial = abort
```

## How the exploration works
* Directions are absolute (0=N 1=E 2=S 3=W), fixed by `START_HEADING` (down in the PDF).
* A junction is detected when LEFT+CENTRE+RIGHT are all black for `ALL3_CONFIRM_MS`, or when LEFT (or
  RIGHT) alone stays black for `SIDE_CONFIRM_MS` (a 3 cm branch keeps it black longer than a wobble).
* The robot then drives on until `JUNCTION_ADVANCE_MS` after the pattern first appeared and records the
  exits: left/right = that side sensor was black for at least `SIDE_MIN_MS` while crossing, straight =
  CENTRE is black after advancing, back = always. Still all three black after advancing = the big circle.
* Every node stores its exits and which exits are already explored. Planner: take an unexplored
  exit here (left, straight, right) -> otherwise BFS over the known graph to the nearest node with
  an unexplored exit -> when none is left, BFS back to the start -> DONE.
* No encoders, so distance = ms of travel at constant speed. A new node is merged with a known
  one (loop closure) only if it has the same exits, its back exit is still unexplored and the
  position agrees within a gate that grows with the accumulated uncertainty.
* On already explored edges the topology is trusted and the position is re-anchored.

## Bring-up checklist (do in this order)
1. Upload, open Serial Monitor at 115200. Send `d` (sensor monitor): on white you want `...`,
   on black `###`. Opposite? flip `LINE_BLACK_IS_HIGH`. Check the left-to-right order (if the module
   has sensitivity trimmers, adjust them so black = 1 and white = 0).
2. Lift the robot, press `s`: both sides must drive forward (else flip the INVERT flags).
3. Tune `PID_KP` / `PID_KD` on a plain straight/curvy line until it follows smoothly.
4. Tune `JUNCTION_ADVANCE_MS` so the robot stops with its wheels centred on a crossing
   (the bar must be past the crossing line but still on the line ahead). Then tune `SIDE_CONFIRM_MS`:
   too small = false junctions from wobble, too large = real side branches missed. Roughly it must be
   shorter than the time to cross a 3 cm line (3 cm / speed) and longer than a typical wobble.
5. Check `CIRCLE_UTURN_MIN_MS` (about 150 degrees of pivot) and `TURN_SPEED`.
6. Full run. Send `m` any time to print the map; it prints automatically at DONE.

## Power notes (important)
* The L298N drops about 2 V, and its on-board 5 V regulator needs the battery at >= 7 V. With two
  Li cells in series (7.4 V nominal) it works but sags as the battery drains.
* Motor stalls can dip the supply and reset the Mega. Add a 470-1000 uF capacitor across the
  battery/L298N supply terminals, near the L298N.
* 2 motors per L298N channel can exceed its 2 A per channel limit when stalled. Keep turns short.

## Test results (host simulation, `test/sim.cpp`)
The map/planner code is compiled on a PC and run on the real graph of `Maze_1.pdf` (20 nodes,
loops, dead ends) with random edge-length errors. Perfect map + return to start in 300 runs:
0-5 % error: 100 %, 8 %: 96 %, 12 %: 85 % (failures end in a clean ERROR, never a silent wrong map).
Build: `g++ -std=c++11 -Itest/stub -Iinclude test/sim.cpp src/maze_map.cpp -o sim && ./sim 0.05 300`

## Known limitation: only 3 sensors
With the sensors about 2 cm apart and a 3 cm line, a small sideways drift turns LEFT or RIGHT black
exactly like a side branch. The code filters this by duration (`SIDE_CONFIRM_MS`) and by requiring
all three black for T/crossings, but it is less reliable than a 5-sensor bar. If the map shows false
or missing junctions, first slow the robot (`BASE_SPEED`), retune the PID, then adjust `SIDE_CONFIRM_MS`.
The sensor logic has not been tested on the real robot yet; the map/planner logic was tested in simulation.
