# Desktop sim

Runs the real autons and Tide tuners on a Mac, with no brain. The code in `src/` and `include/tide/`
compiles with the host compiler against fake PROS and LemLib headers. A simple physics model moves the
robot.

```bash
make -C tools/sim run AUTON=X        # any name from the Auton page; no name lists them
make -C tools/sim check              # tuners must find known offsets on a test robot
```

`run` prints each LemLib motion as it ends ("TIMEOUT" stands out), the final true and odom pose, and writes:
- `tools/sim/out/<auton>.svg`: the path over `docs/field-map.svg` (blue = robot, pink = odom)
- `tools/sim/out/<auton>.json`: the pose every 10 ms, the motion and claw log, every piece's moves, Toggle faces and the score; the browser viewer reads the same file

Options go straight to the binary: `tools/sim/out/sim X --blue --time 60000` (skills length).

## In the browser
`make -C tools/sim web` builds the same sim for the browser with Emscripten (`brew install emscripten`)
into `tools/sim/out/web/sim.js`: one file with the WebAssembly inlined. The tide-sim viewer
(`~/Developer/Web/Tail/tide-sim`, TailEngine) loads it, lists the autons, runs one, and replays it
on the field with a timeline and the motion log. `bun run sim` there rebuilds 654T's copy into its
`public/sims/`. A 15 s auton takes about 40 ms.

## Pieces
| Where | What |
|---|---|
| `tools/sim/fake/` | PROS, LemLib and `tide/screen_ui.hpp` stand-ins, found ahead of `include/` |
| `tools/sim/world.hpp` | Task scheduler on sim time (threads on the desktop, Asyncify fibers in the browser), drive and joint physics, wall collisions, distance-sensor ray casts |
| `tools/sim/game.hpp` | This season's game (Override): starting Pins, Cups, Goals and Toggles, claw pickups, Toggle spinning, scoring. Swap it out next season |
| `tools/sim/main.cpp` | Runs `initialize()` then the picked auton through `autonomous()` |
| `tools/sim/check.cpp` | `tide::trackingOffset` and `Odom::measureMounts` on a test robot with known offsets |
| `src/sim.hpp` | 654T's physical truth: footprint, real sensor mounts, winch gearing, what the claw and Toggle wheel do, the preload. The brain build never sees it |

Drivetrain size, wheels, rpm and tracking wheel come from `robot.hpp`. `src/sim.hpp` only sets what
the code doesn't know, or what it has wrong, such as a mount that hasn't been measured yet. The sim
reads its own copy of each value, so a wrong value in `robot.hpp` shows up as odom (pink) drifting
away from the robot (blue).

## Model
- Time only moves while every task is waiting in `pros::delay`, so runs come out the same every
  time. A 15 s auton finishes in well under a second.
- Each drive side reaches its voltage's share of top speed with a 0.25 s time constant
  (`World::driveTau`). Coast, brake and hold slow it down at different rates.
- The robot is a rectangle that stops at the perimeter. Its wheels stall there, so `pushUntilStall`
  and `wallReset` work.
- Motions use the robot's own `ControllerSettings` gains and exit conditions, with simplified
  LemLib logic (boomerang `moveToPose`, motion queueing, `minSpeed` early exits). Path shapes are
  close to LemLib's, not exact.
- Odom uses LemLib's tracking-wheel and IMU math.
- Game pieces are circles. The robot shoves loose ones out of its footprint, except in the channel
  between its claw and the nearest edge. The claw picks up one Pin and one Cup within reach of its
  tool point when `src/sim.hpp` says it's gripping and low enough. It drops them when it opens,
  stacking them on a Goal if the tool point is within 2.5" of one.
- A Toggle turns to its next face while the spinner motor runs with the robot's front against it.
  The color sensor reads the hue of the face it's touching.
- Score: 5 per alliance-coloured top half stacked on a Goal, 10 per owned yellow one. A Quadrant's
  Toggle owns its yellow halves; the Midfield Goal's go to red while our robot is in Midfield.

## Doesn't model
Wheel slip, battery sag, motor heat, how the real drive responds (`driveTau` is a guess), sensor
noise, opponents, or the brain screen. For game pieces it doesn't model collisions with Goals,
pieces tipping over, distance sensors seeing pieces, a Cup's orientation (every stacked Pin shows its
top half), match loads or the auton bonus. The winch gearing and gravity, the footprint, and the
claw's reach, height and preload in `src/sim.hpp` are guesses.
