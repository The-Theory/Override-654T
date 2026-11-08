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
- `--setup right back dheading`: the robot really sits this far off where the auton's first `setPose` says,
  relative to the robot: `back` is off the wall it faces, so one sweep fits any start
- `--distance-scale k`: distance sensors read k times the true range; `--jitter in [--seed n]`: Cup + Pin
  spots land up to that far off their nominal spot
- `--tune "<Tune page name>" [--start x y heading]`: runs a tuner on the sim robot instead of an auton,
  set down mid-field or at --start
- `make -C tools/sim robust AUTON=X`: X over a grid of those; every line should score the same.
  `ARGS="--time 60000"` passes options through, for Skills

## In the browser
```bash
make -C tools/sim view               # builds the browser sim, serves the viewer on localhost:5195
```
Needs Emscripten and Bun (`brew install emscripten oven-sh/bun/bun`). `make -C tools/sim web` alone
rebuilds `tools/sim/web/public/sim.js`: the same sim as one file with the WebAssembly inlined; a 15 s
auton runs in about 40 ms. The viewer (`tools/sim/web`, React + Vite) lists the autons, runs one, and
replays it:
- the field with the robot, where odom thinks it is, and every piece, Goal and Toggle; hover any of them
  for what it is and where
- the robot's position and odom error under the field, and a timeline of motions (red if they timed
  out) and claw actions (yellow); drag to scrub
- the score, whether the auton finished in time, and every step in order; click a step to jump to it
- Space plays and pauses, arrows step 0.1 s (1 s with shift)

## Pieces
| Where | What |
|---|---|
| `tools/sim/fake/` | PROS, LemLib and `tide/screen_ui.hpp` stand-ins, found ahead of `include/` |
| `tools/sim/world.hpp` | Task scheduler on sim time (threads on the desktop, Asyncify fibers in the browser), drive and joint physics, wall collisions, distance-sensor ray casts |
| `tools/sim/game.hpp` | This season's game (Override): starting Pins, Cups, Goals and Toggles, claw pickups, Toggle spinning, scoring. Swap it out next season |
| `tools/sim/main.cpp` | Runs `initialize()` then the picked auton through `autonomous()` |
| `tools/sim/web/` | Browser viewer |
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
  (`World::driveTau`), longer with the lift up (`World::liftTau`). Coast, brake and hold slow it down at
  different rates. Friction (`World::driveStiction`, 1.2 V) holds it at rest under that voltage and opposes
  it while moving, braking included, so a PID that fades out near its target stalls short, like on carpet.
- The robot is a rectangle that stops at the perimeter. Driven into a wall at an angle, it pivots on the
  corner touching it until the bumper sits flush, like slamming into the wall to square up; the wall
  pushes it back along its heading, since the wheels don't slip sideways. Once flush
  its wheels stall, so `pushUntilStall` and `wallReset` work. It doesn't slide along the wall.
- Motions use the robot's own `ControllerSettings` gains and exit conditions, with simplified
  LemLib logic (boomerang `moveToPose`, motion queueing, `minSpeed` early exits). Path shapes are
  close to LemLib's, not exact.
- Odom uses LemLib's tracking-wheel and IMU math.
- Game pieces are circles. The robot shoves loose ones out of its footprint, except in the channel
  between its claw and the nearest edge. The claw closes on one Pin and one Cup within reach of its
  mouth (the back bumper) when `src/sim.hpp` says it's gripping, and its distance sensor reads the
  nearest loose piece from there. It drops them when it opens, stacking them on a Goal if the tool
  point is within 2.5" of one.
- Loaders are blocks the robot and distance beams stop on. What the drive team drops in comes from
  `sim::game::matchLoads` in `src/sim.hpp`, per auton and in order (`PIN`, `CUPPIN`, `CUP`); the next
  one lands in the bottom opening 0.3 s after the last is pulled out. Match autons get none (SG11).
- A Toggle turns to its next face while the spinner motor runs with the robot's front against it.
  The color sensor reads the hue of the face it's touching.
- Score: 5 per alliance-coloured top half stacked on a Goal, 10 per owned yellow one. A Quadrant's
  Toggle owns its yellow halves; the Midfield Goal's go to red while our robot is in Midfield.

## Doesn't model
Wheel slip, battery sag, motor heat, how the real drive responds (`driveTau` and `driveStiction` are guesses), sensor
noise, opponents, or the brain screen. For game pieces it doesn't model collisions with Goals,
pieces tipping over, distance sensors other than the claw's seeing pieces, a Cup's orientation (every stacked Pin shows its
top half) or the auton bonus. The winch gearing and gravity, the footprint, and the
claw's reach, height and preload in `src/sim.hpp` are guesses.
