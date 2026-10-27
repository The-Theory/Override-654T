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



Next, add `ctl.update();` in your driver loop, such as before your `pros::delay()` call.

So, a basic version of your operator function would look like this:

```cpp
void opcontrol() {
    tide::Control ctl(controller);

    while (true) {
        ctl.update();
        pros::delay(25);
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

Now, pressing `R1` turns our intake on, and pressing it again turns it off. However, when actually driving, our driver would probably appreciate being able to use `R1` and `R2` to change directions. We can do this as such:

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
bool cascadeDown() { return winch.get_position() < 50; }

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
tide::Mechanism winchPID(winch, winchEncoder, {.kP = 30, .kI = 0.5, .kD = 100, .windup = 20}, {.min = 0, .max = 1100});
```

The second braces are the hard stops. Targets get clamped into them, so the mechanism is never asked to go somewhere it can't reach. Leave either one out if that side has no stop.

Call `winchPID.start()` in `initialize()`. Then move it with `moveTo()` / `moveBy()`, let it go limp with `release()`, or drive it by hand with `bidir`:

```cpp
ctl.when(L).bidir(winchPID);
```

>Unlike `bidir` on a motor, letting go holds the mechanism where it is instead of stopping it.

- `moveBy(200)` moves from where the mechanism is now, not from its last target.
- Both take an optional voltage cap for slower moves: `winchPID.moveTo(800, 6000)`. The cap applies to the PID's push, and `kG` still gets added on top, so a slow lift can still hold its load.

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

- `odom.wallReset(tide::Wall::RED)` corrects the one coordinate facing that wall, using whichever sensor points at it. It skips the reset (returns `false`) if no sensor is within 15° of square or the wall is over 40" away.
- `odom.moveBy(-tide::TILE)` moves relative to where the robot is: forward, then optional right and turn (`odom.moveBy(12, 12, 90)`), all from the current pose. Negative forward backs up; heading holds unless `turn` is given. A sideways offset curves there, since tank can't strafe. Chained relative moves stack their small errors, so follow with an absolute move or `wallReset` when it matters.
- `odom.reachPoint(goal)` drives straight at a spot and stops with the **tool** on it instead of the tracking center. It backs in if the tool is on the back. `odom.reachPose(goal, EAST)` does the same but arrives facing a heading. Set where the tool sits with `.toolRight` / `.toolForward` in the settings: `tide::Odom odom(chassis, ..., {mounts}, {.toolForward = -6});`
- `odom.turnBy(-90)` turns relative to the current heading. Positive is clockwise, and it turns that way even past 180°.
- `odom.pushUntilStall(6500)` drives into a wall or Toggle at a voltage and stops once the tracking wheel stops. `odom.pushUntilStall(6500, 1500, 250)` then keeps pushing at full voltage for 250 ms. On a soft wall that bends and stops the robot early, that steady push still turns the Toggle over.

>Only reset while stopped, with the beam on bare wall, not a Loader or Scoring Objects on the wall.

`tide::dir` holds the LemLib headings (`NORTH = 0`, `EAST = 90`, ...), pulled in with `using namespace tide::dir;`. `tide::TILE` is the tile pitch (23.4").




---





## Brain Screen

`tide/screen.hpp` replaces the default **Brain** screen with three pages, picked with the buttons along the top:

- **Auton**: on the left, the alliance (tap to flip Red / Blue) and up to 2 on/off options; past a divider, one button per auton (2 columns, 3 past 6 autons, 9 max).
- **Tune**: one button per tuner on the left, the last result in a panel on the right.
- **Info**: live `"Key\tvalues"` lines you supply, refreshed 10 times a second; keys sit in a dim column so values line up.

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
tide::autotune("winch", tide::mechanismPlant(winchPID), {.setpoint = 400, .amplitude = 4000, .maxDeviation = 200});
tide::autotune("angular", tide::angularPlant(leftMotors, rightMotors, imu), {.setpoint = 0, .amplitude = 4000, .maxDeviation = 45});
```

Results print to the terminal and come back in `TuneResult::summary` (show it on the **Brain** with a screen tuner): only the gains to paste: `kP / kI / kD / kG` for a `tide::Gains` plant, or `kP / kI / kD` already scaled for **LemLib** `ControllerSettings` on the angular and lateral plants.

- `amplitude` [mV] must beat static friction and the holding voltage. If it reports no oscillation, raise it.
- `maxDeviation` aborts the tune if it swings too far, protecting hard stops.
- `rule` picks how aggressive: `NO_OVERSHOOT` (default), `SOME_OVERSHOOT`, `CLASSIC`.
- Anything can be tuned by giving a `tide::Plant` with `measure()` (position) and `apply(mV)`.

>The gains are a starting point. The chassis especially (friction, wheel slip) usually needs a little hand-tuning afterward.
