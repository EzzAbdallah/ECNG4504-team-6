# ECNG4504 Maze Robot - Phase 0 (training run)

Arduino Mega 2560 + L298N + 5-channel line sensor, PlatformIO, C++ only.
The robot follows the line (PID), stops at every node, records it in a graph, explores the
**whole** maze (loops and dead ends included) and returns to the start circle. No shortest path
and no BLE yet (Phase 1).

## Code layout (one module = one .h + one .cpp)
| Module | Job |
|---|---|
| `config.h` | **All pins and all tunable numbers.** Edit here only. |
| `motors` | L298N driver. Register-level: Timer3/Timer4 fast PWM on D5/D6, direction on PORTA (D22-D25). |
| `line_sensor` | Reads the 5 sensors -> bit mask, count, weighted position (-2..+2). |
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
| Line sensor S0..S4 (left..right) | D30..D34 | sensor VCC=5V, GND common |
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
* At each junction the robot drives `JUNCTION_ADVANCE_MS` forward and records which exits exist
  (left/right from the outer sensors, straight from the centre sensors after advancing, back always).
  Still all-black after advancing = the big circle.
* Every node stores its exits and which exits are already explored. Planner: take an unexplored
  exit here (left, straight, right) -> otherwise BFS over the known graph to the nearest node with
  an unexplored exit -> when none is left, BFS back to the start -> DONE.
* No encoders, so distance = ms of travel at constant speed. A new node is merged with a known
  one (loop closure) only if it has the same exits, its back exit is still unexplored and the
  position agrees within a gate that grows with the accumulated uncertainty.
* On already explored edges the topology is trusted and the position is re-anchored.

## Bring-up checklist (do in this order)
1. Upload, open Serial Monitor at 115200. Send `d` (sensor monitor): on white you want `.....`,
   on black `#####`. Opposite? flip `LINE_BLACK_IS_HIGH`. Check the left-to-right order.
2. Lift the robot, press `s`: both sides must drive forward (else flip the INVERT flags).
3. Tune `PID_KP` / `PID_KD` on a plain straight/curvy line until it follows smoothly.
4. Tune `JUNCTION_ADVANCE_MS` so the robot stops with its wheels centred on a crossing
   (the bar must be past the crossing line but still on the line ahead).
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
