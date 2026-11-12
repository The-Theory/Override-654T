# _Tide_ Walkthrough

A controller library for **VEX** by Theo Hallgren





---





## Intro

**Tide** is a library for **PROS** to easily handle controller bindings and the general workflow of controller input to robot output. 

The source is currently available at https://github.com/The-Theory/Override-654T/tree/main/include/tide, though it will be separated in the future to a standalone repo available as a **PROS** depot.

**Tide**'s only dependency is **PROS**, meaning that you can use **LemLib**, **EZ-Template**, something else, or nothing at all. As long as **PROS** is installed, **Tide** will work.

>For this walkthrough, I will be using **LemLib**, though it should have a minimal impact on the showcases. 





---





## Setup

**Tide** needs two lines at the top of your main file (or wherever you want to define your bindings):
```cpp
#include "tide/control.hpp"
using namespace tide::btn;
```
>All this does is actually include **Tide** in your program, and also add access to _button shortcuts_. These shortcuts will be very useful in defining your first binds, which we'll go over soon.



Next, add the following at the top of your operator control function:
```cpp
tide::Control ctl(controller);
```
`controller` here is a `pros::Controller`, and is the controller you'll be binding. 

>Support for two controllers is work in progress, though for now you can just create two `tide::Control` objects.



Next, add `ctl.update();` in your driver loop. It waits 10 ms each call, so the loop needs no `pros::delay()` of its own.

So, a basic version of your operator function would look like this:

```cpp
void opcontrol() {
    tide::Control ctl(controller);

    while (true) {
        ctl.update();
    }
}
```





---





## First Binds

A very common mechanism to start with is the _intake_, a usually spinning mechanism powered by a motor or two. 

In this case, our intake relies on a `pros::Motor`, though a `pros::MotorGroup` would work just as well. Assuming it's called `intakeMotor`, we can add the following line after `tide::Control ctl(controller)`:

```cpp
ctl.when(R1).hold(intakeMotor);
```

This line assigns the `R1` button to spin our intake forward. But holding `R1` when testing is annoying, so let's make it a toggle instead:

```cpp
ctl.when(R1).toggle(intakeMotor);
```

Now, pressing `R1` turns our intake on, and pressing it again turns it off. The motor is only set on a press, so a macro using the same motor isn't overridden in between. However, when actually driving, our driver would probably appreciate being able to use `R1` and `R2` to change directions. We can do this as such:

```cpp
ctl.when(R).bidir(intakeMotor);
```

**Tide** understands `R` as representing the `R1` and `R2` pair, meaning we don't have to worry about any of that logic.





---





## Multiple Binds

Now, we will likely need to bind more than one mechanism. Let's do a claw, specifically the pivoting of the claw. 

Our claw has two main positions, independent of the closing state: _up_ and _down_.

So let's define two _macros_:
```cpp
void clawPivotUp()   { clawPivot.move_absolute(CLAW_PIVOT_UP, CLAW_PIVOT_RPM); }
void clawPivotDown() { clawPivot.move_absolute(CLAW_PIVOT_DOWN, CLAW_PIVOT_RPM); }
```

These two functions define what angle to keep our claw at. Now we want to be able to switch between these states. Here's how we would bind these functions to `A` and `B`, while keeping our previous bind:

```cpp
tide::Control ctl(controller);
ctl.when(R).bidir(intakeMotor)
   .when(A).run(clawPivotUp)
   .when(B).run(clawPivotDown);
```

We can use a _return chain_ to keep adding lines, adding however many binds we want. 

> Notice that only the last line ends with a semicolon (`;`).





---





## Combos

Let's imagine we have a mechanism that can be in four different states, and we need to be able to switch between them freely. Taking four buttons for just one mechanism can cause overlap with other controls, and make the bindings pretty confusing.

So we can give all of that mechanism's bindings the same modifier button, like a _shift_ key. It's easiest to show:

```cpp
ctl.when(A + UP).run(stateOne)
   .when(A + RIGHT).run(stateTwo)
   .when(A + DOWN).run(stateThree)
   .when(A + LEFT).run(stateFour);
```

By doing this, our mechanism is based on the `A` button, with the state selected by the directional buttons. This allows the directional buttons to be assigned to other mechanisms, while still allowing them to control our main mechanism when the `A` button is held down.

> To trigger a combo, _both_ buttons need to be held at the same time, in either order.

While `A` is held, bindings on those plain directional buttons won't fire, so the combo always wins. For the same reason, avoid giving `A` a binding of its own: it fires the moment `A` goes down, before **Tide** can know a directional button is coming.





---





## Modifiers

So far, every bind fires the moment its button goes down. _Modifiers_ change exactly when a bind fires. They go between `when` and the action, and you can stack as many as you need, in any order.

#### Releasing

```cpp
ctl.when(B).onRelease().run(clawPivotDown);
```

`onRelease()` fires the bind when `B` is let go, instead of when it's pressed.

#### Holding and Tapping

```cpp
ctl.when(Y).heldUnder(300).run(clawPivotUp)
   .when(Y).heldFor(300).macro(scoringMacro);
```

`heldFor(300)` waits until `Y` has been held for 300 milliseconds before firing. `heldUnder(300)` fires when `Y` is let go, but only if it was held for less than 300 milliseconds. Together, a quick tap of `Y` raises the claw, while holding it runs the scoring macro, giving one button two jobs.

> Adding `onRelease()` to a `heldFor()` bind makes it fire on release instead, but only if the button was held long enough.

#### Repeating

```cpp
ctl.when(UP).repeat(100).run(liftStepUp)
   .when(DOWN).repeat(100).run(liftStepDown);
```

`repeat(100)` fires once when `UP` is pressed, and then again every 100 milliseconds for as long as it stays held. If `liftStepUp` raises our lift by a small amount, a quick press nudges it once, while holding `UP` raises it smoothly.

#### Conditions

```cpp
bool cascadeDown() { return winchMotors.get_position() < 50; }

ctl.when(R).onlyIf(cascadeDown).bidir(intakeMotor)
   .when(A).onlyIf(cascadeDown).macro(pickupMacro);
```

`onlyIf` takes any function that returns `true` or `false`. Here, the intake won't spin, and the pickup macro won't run, unless the cascade is down.

> Most binds check the condition the moment they would fire. Pressing `A` while the cascade is up does nothing, even if it lowers while `A` is still held. `hold` and `bidir` check it the whole time instead, so the motor stops as soon as the condition turns `false`.

#### Rumble

```cpp
ctl.when(X).rumble(".").macro(scoringMacro);
```

`rumble` vibrates the controller whenever the bind fires, so our driver can feel that the macro started without looking away from the field. Use `.` for a short rumble, `-` for a long one, and spaces for pauses, up to 8 characters.

> **PROS** marks controller rumble as beta, and the controller only updates so fast, so avoid rumbling on binds that fire often, like `repeat()`.





---





## The Modular Structure of _Tide_

You may have noticed that all of our binds start with `.when`, which seems really wasteful when we could just pass it as a parameter instead into our action functions. 

To showcase the reason for `.when`, here is one binding written both ways. The first is how it would look if every setting were a parameter, and the second is how **Tide** writes it:

```cpp
// Hold A for half a second, fire on release, only while the claw is open
ctl.run(onRelease(onlyIf(heldFor(A, 500), clawOpen)), fn);   // wrappers nest inside out
ctl.when(A).heldFor(500).onlyIf(clawOpen).onRelease().run(fn); // steps read left to right
```

Both describe the same binding. However, the first has to be read from the inside out, while the second reads left to right, like a sentence. 

Now how often you need to _run a function when you release a button after it's been held for half a second if your claw is open_ is questionable, but it shows how **Tide** stays readable as your bindings grow more specific.




---





## PID Mechanisms

For arms and lifts with a rotation sensor, `tide::Mechanism` holds the motor at a target using PID instead of the motor's built-in encoder:

```cpp
tide::Mechanism winch(winchMotors, winchEncoder, {.kP = 30, .kI = 0.5, .kD = 100, .windup = 20}, {.min = 0, .max = 1100});
```

The second braces are the hard stops. Targets get clamped into them, so the mechanism is never asked to go somewhere it can't reach. Leave either one out if that side has no stop.

Call `winch.start()` in `initialize()`. Then move it with `moveTo()` / `moveBy()`, let it go limp with `release()`, or drive it by hand with `bidir`:

```cpp
ctl.when(L).bidir(winch);
```

>Unlike `bidir` on a motor, letting go holds the mechanism where it is instead of stopping it.

- `moveBy(200)` moves from where the mechanism is now, not from its last target.
- Both take an optional voltage cap for slower moves: `winch.moveTo(800, 6000)`. The cap applies to the PID's push, and `kG` still gets added on top, so a slow lift can still hold its load.

For a lift or arm, set `.kG` to the voltage that just holds the load still. It's added to every PID output, so the integral doesn't have to hold the weight. The auto-tuner below measures it for you. When the target is the stop that gravity pushes it into, `kG` is skipped, because the stop already holds the load.

### Zeroing

| Call | Zero is… | Survives a program restart | Survives a power cycle |
|---|---|---|---|
| `start()` | wherever the mechanism is when called | no | no |
| `resume()` | whatever the sensor counted last run | yes | no |
| `startAbsolute(zeroAngle)` | from the sensor's absolute angle | yes | yes |

`startAbsolute()` only works if the sensor turns less than one revolution over the mechanism's full travel (an arm pivot, not a winch spool). To get `zeroAngle`, put the mechanism at its zero and read `rotation.get_angle() / 100.0`.




---





## Odom Helpers

`tide/odom.hpp` sits on top of a **LemLib** chassis and swaps battery- and tile-dependent moves for ones that end in the same place every time:

```cpp
tide::Odom odom(chassis, leftMotors, rightMotors, verticalEncoder, {
	// sensor          right [in]  forward [in]  facing
	{leftDistance,     -6,         2,            WEST},
	{middleDistance,   0,          7,            NORTH},
});
```

Each distance sensor is listed with where it sits relative to the tracking center and which way it points (`tide::dir` headings, relative to the robot's front).

- `odom.wallReset(tide::Wall::RED)` corrects the one coordinate facing that wall, using whichever sensor points at it. It waits until the robot has stopped moving and turning, takes the median of 5 readings, and prints what it changed. It skips the reset, prints why, and returns `false` if no sensor is within 15° of square, the wall is over 40" away, or the reading would move the pose more than 6" (the beam hit a robot or piece).
- `odom.moveBy(-tide::TILE)` moves relative to where the robot is: forward, then optional right and turn (`odom.moveBy(12, 12, 90)`), all from the current pose. Negative forward backs up; heading holds unless `turn` is given. A sideways offset curves there, since tank can't strafe. Chained relative moves stack their small errors, so follow with an absolute move or `wallReset` when it matters.
- `odom.reachPoint(goal)` drives straight at a spot and stops with the **tool** on it instead of the tracking center. It backs in if the tool is on the back. `odom.reachPose(goal, EAST)` does the same but arrives facing a heading. Set where the tool sits with `.toolRight` / `.toolForward` in the settings: `tide::Odom odom(chassis, ..., {mounts}, {.toolForward = -6});`
- `odom.turnBy(-90)` turns relative to the current heading. Positive is clockwise, and it turns that way even past 180°.
- `odom.pushUntilStall(6500)` drives into a wall or Toggle at a voltage and stops once the tracking wheel stops. `odom.pushUntilStall(6500, 1500, 250)` then keeps pushing at full voltage for 250 ms. On a soft wall that bends and stops the robot early, that steady push still turns the Toggle over.

>Only reset with the beam on bare wall, not a Loader, Toggle or Scoring Objects on the wall.

### Steps for routines

Coordinates get the robot close; sensors finish the job. Routines are built from small steps instead of one function with a flag per field:

- `odom.approach(goal, EAST, 8)` lands the tool 8" short of `goal`, facing `EAST`, and waits until it's stopped. It drives forwards or backwards, whichever turns less, so it never loops around. `odom.approachStraight(piece, 8)` does the same facing straight at the target from where the robot is.
- `odom.creep(-3000, [] { return clawSees(); })` drives straight at a low voltage until the condition is true (or the timeout), for the last inches onto something a sensor can see. `[] { return false; }` just pushes for the timeout.
- `odom.scan(clawMount, 20, 1.55)` turns slowly through ±20° reading a distance sensor and returns where each object it passed sits on the field. Pick the one nearest where a piece should be, then `approachStraight` to it: a shifted piece or robot still lines up, and neighbors aren't mistaken for it.
- `odom.turnTo(EAST)` turns to a heading.

```cpp
bool grab(tide::Point piece) {
	openClaw();
	odom.approachStraight(piece, 8);
	odom.approachStraight(spotted(piece), 8);			// nearest scanned object
	const bool seen = odom.creep(-3000, clawSeesPiece);
	closeClaw();
	return seen;
}
```

### Planned moves

Once the drive's feedforward is measured (see [Auto-Tuning](#auto-tuning)), `approach`, `turnTo` and straight `moveBy` stop using LemLib's PID and follow a speed plan instead: speed up at a set rate, cruise, slow down to land on the end stopped. Every 10 ms the robot knows where it should be and how fast; the feedforward (`kS + kV·v + kA·a`) supplies most of the voltage, and feedback fixes the rest. The feedback gains come from `kV` and `kA` (critically damped at `.bandwidth`), so there's nothing to tune.

```cpp
tide::Odom odom(chassis, ..., {mounts}, {
	.lateral       = {1205, 166.5, 41.7},	// Tune page: FF down
	.lateralLoaded = {1205, 166.5, 58.2},	// FF up
	.angular       = {1205, 16.6, 4.15},
	.angularLoaded = {1205, 16.6, 5.80},
	.load          = [] { return winch.position() / LIFT_LOADED; },
});
```

`.load` says how far toward the loaded gains the robot is right now (0 to 1), so a raised lift, which adds inertia, gets its own feedforward and stiffer feedback on its own, even mid-move. `odom.profiledDrive(24)` and `odom.profiledTurn(90)` run a planned move directly. Leave `.lateral` unset and everything uses LemLib's moves as before.

### Recording a driver run

`tide::Recorder` (`tide/record.hpp`) records a driver run to the SD card and plays it back as an auton. Every 10 ms it saves where odom thinks the robot is, plus each mechanism you list as a `tide::Channel`. A Channel is a name, how to read it, and how to set it on playback:

```cpp
tide::Recorder recorder(odom, chassis, {
	{"claw",  [] { return double(claw.get_voltage()); }, [](double mv) { claw.move_voltage(mv); }, 1000},
	{"winch", [] { return winch.position(); },           [](double at) { winch.moveTo(at); },     20},
});

recorder.start("skills");	// from a known start pose, in driver control
recorder.stop();			// saves /usd/skills.txt
recorder.replay("skills");	// in an auton
```

It doesn't record controller input: replaying sticks would drift a little more every second. Playback sets the pose to where the recording started, then `odom.track` follows the path. Track is a RAMSETE path follower: the feedforward supplies the voltage, and RAMSETE steers back onto the path when the robot gets pushed off, so it needs planned moves measured first. Each channel is set again as the path reaches the point where it changed. The last number in a channel is the smallest change worth replaying, so sensor noise isn't replayed.

A channel's setter can take its time. The path waits while it runs, so it can, for example, look for a piece that sits somewhere else on this field before the claw closes. Square the robot on a wall before `replay` too, so a crooked setup doesn't bend the whole path.

## Field

`tide/field.hpp` holds the field geometry the rest of Tide uses, the same every season:

- `tide::dir` holds the LemLib headings (`NORTH = 0`, `EAST = 90`, ...), pulled in with `using namespace tide::dir;`.
- `tide::TILE` is the tile pitch (23.4"), `tide::Point` a field position in inches, `tide::Wall` the four perimeter walls.

Each game gets its own header of named positions. `tide/override.hpp` names the red-side Override elements under `tide::override`: `goal::`, `cuppin::`, `cross::` and `toggle::`. Pull them in with `using namespace tide::override;` and write `odom.reachPoint(goal::MIDFIELD)`. The map of every name is `docs/field-map.svg`.




---





## Brain Screen

`tide/screen.hpp` replaces the default **Brain** screen with three pages, picked with the buttons along the top:

- **Auton**: on the left, the alliance (tap to flip Red / Blue) and up to 2 on/off options; past a divider, one button per auton (2 columns, 3 past 6 autons, 9 max).
- **Tune**: one button per tuner on the left (two columns past 3, so up to 6 fit), the last result in a panel on the right.
- **Info**: live `"Key\tvalue\tvalue"` lines you supply, refreshed 10 times a second; keys sit in a dim column and each tab starts a fixed column (up to 3), so changing digits don't shift the line.

Flat navy theme, 6 px gaps, 56 px rows; the selected button is light blue.

```cpp
void x_auton(tide::Alliance side) { ... }
std::string tune_winch() { return tide::autotune(...).summary; }

const std::vector<tide::screen::Auton> autons = {{"X", x_auton}};
const std::vector<tide::screen::Tuner> tuners = {{"Winch", tune_winch}};

void initialize() { tide::screen::init(autons, tuners, robotInfo, {"Nanny"}); }
void autonomous() { tide::screen::runAuton(); }	// Selected auton with selected alliance
// In opcontrol: .when(RIGHT).run(tide::screen::runTuner)
```

`tide::screen::alliance()` reads the selected alliance anywhere, `tide::screen::option("Nanny")` an option. `tide::screen::inches(sensor)` and `tide::screen::hottest(motors)` help format info text.

>Tuners only run from the controller, never from a tap on the screen, so nobody's hand is on the robot when it starts moving.

---





## Auto-Tuning

`tide/autotune.hpp` runs a relay auto-tune: it drives full swing one way and the other around a setpoint, measures the oscillation the system settles into, and turns that into PID gains (Ziegler-Nichols). It also finds the voltage that holds the load (`kG`).

```cpp
tide::autotune("winch", tide::mechanismPlant(winch), {.setpoint = 400, .amplitude = 4000, .maxDeviation = 200});
tide::autotune("angular", tide::angularPlant(leftMotors, rightMotors, imu), {.setpoint = 0, .amplitude = 4000, .maxDeviation = 45});
tide::autotune("lateral", tide::lateralPlant(leftMotors, rightMotors, tracking, wheelDiameter),
	{.setpoint = 0, .amplitude = 7000, .maxDeviation = 12, .hysteresis = 0.25, .rule = tide::TuneRule::SOME_OVERSHOOT});
```

Results print to the terminal and come back in `TuneResult::summary` (show it on the **Brain** with a screen tuner): only the gains to paste: `kP / kI / kD / kG` for a `tide::Gains` plant, or `kP / kI / kD` already scaled for **LemLib** `ControllerSettings` on the angular and lateral plants.

- `amplitude` [mV] must beat static friction and the holding voltage. If it reports no oscillation, raise it. For the drive, swing big (~7000): a small swing is mostly friction, which reads as a sluggish plant and gives slow gains.
- `maxDeviation` aborts the tune if it swings too far, protecting hard stops.
- `rule` picks how aggressive: `NO_OVERSHOOT` (default), `SOME_OVERSHOOT`, `CLASSIC`. `NO_OVERSHOOT` creeps into its target, so a drive can stall short on friction; `SOME_OVERSHOOT` suits lateral.
- `tide::trackingOffset(left, right, imu, tracking, wheelDiameter)` spins in place and returns the vertical tracking wheel's offset for `lemlib::TrackingWheel`, so you don't have to find the turning center by eye.
- `odom.measureMounts(i, j, apart)` finds where each distance sensor sits (right and forward of the tracking center): it backs into a wall to square up, pulls off, then sweeps each sensor ±15° across that wall, spinning in place. Give it two sensors facing opposite ways and the tape-measured distance between their faces. It needs a flat back bumper (or pass a positive `squareMv` to use the front).
- `odom.measureBumpers(side)` finds how far the front and back bumpers are from the tracking center: it squares each bumper on a wall, backs off, and turns a sideways sensor (index `side`) to the wall. Needs measured mounts. Start poses flush on a wall and a tool at a bumper then come from measurements, not guesses.
- `tide::characterize("lateral", plant)` measures a plant's feedforward (`kS` friction, `kV` speed, `kA` inertia) from a slow voltage ramp and a hard step each way, and prints a `tide::Feedforward` to paste. Run it once per pose that changes the robot (lift down, lift up): `kA` moves the most. About 2 ft clear front and back.
- Anything can be tuned by giving a `tide::Plant` with `measure()` (position) and `apply(mV)`.

### Calibration order

Each step uses the ones before it:

1. `trackingOffset` (tracking wheel offset)
2. `measureMounts` (distance sensors)
3. `measureBumpers` (start poses, tool point)
4. `characterize` with the lift down, then up (planned moves)

>The gains are a starting point. The chassis especially (friction, wheel slip) usually needs a little hand-tuning afterward.
