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
- `tools/sim/out/<auton>.json`: the pose every 10 ms plus the motion log, for the browser viewer to come

Options go straight to the binary: `tools/sim/out/sim X --blue --time 60000` (skills length).

## Pieces
| Where | What |
|---|---|
| `tools/sim/fake/` | PROS, LemLib and `tide/screen_ui.hpp` stand-ins, found ahead of `include/` |
| `tools/sim/world.hpp` | Task scheduler on sim time, drive and joint physics, wall collisions, distance-sensor ray casts |
| `tools/sim/main.cpp` | Runs `initialize()` then the picked auton through `autonomous()` |
| `tools/sim/check.cpp` | `tide::trackingOffset` and `Odom::measureMounts` on a test robot with known offsets |
| `src/sim.hpp` | 654T's physical truth: footprint, real sensor mounts, winch gearing. The brain build never sees it |

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

## Doesn't model
Wheel slip, battery sag, motor heat, how the real drive responds (`driveTau` is a guess), sensor
noise, game objects, Toggles (the color sensor sees nothing), opponents, or the brain screen.
The winch gearing and gravity in `src/sim.hpp` are guesses.
