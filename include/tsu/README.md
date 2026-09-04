# TsuControl

Controller bindings for 654T Tsunami. Declare what each button does once,
then poll everything with a single `update()` call per opcontrol loop.

```cpp
#include "tsu/control.hpp"
using namespace tsu::btn;

void opcontrol() {
    tsu::TsuControl ctl(controller);
    ctl.bidir(intakeMotor, R)      // R1 forward, R2 reverse
       .bidir(winch, -L)           // L2 forward, L1 reverse
       .toggle(clawPivot, A);

    while (true) {
        ctl.update();
        chassis.curvature(ctl.axis(LY), ctl.axis(RX));
        pros::delay(25);
    }
}
```

## Names

`using namespace tsu::btn;` gets you the bare names. Qualify them as
`tsu::btn::R2` instead if a name collides with something.

| Kind | Names |
| --- | --- |
| `Button` | `L1 L2 R1 R2 UP DOWN LEFT RIGHT A B X Y` |
| `Pair` | `L R DPAD_V DPAD_H FACE_V FACE_H` |
| `Axis` | `LX LY RX RY` |

A `Pair` is two buttons driving one mechanism in opposite directions. The
first name in the pair drives forward, so `R` is R1 forward and `R2` reverse.
Negating flips that: `-R` is R2 forward. Pairs can also be written out, as in
`{UP, R2}`.

## Bindings

Every binding returns the `TsuControl` so they chain, and each runs in the
order it was registered.

| Binding | Behavior |
| --- | --- |
| `bidir(motor, pair, mv = 12000)` | Forward while the pair's first button is held, reversed while the second is, stopped otherwise. Forward wins if both are down. |
| `hold(motor, button, mv = 12000)` | Runs while held, stops on release. |
| `toggle(motor, button, mv = 12000)` | Each new press flips the motor on or off. |
| `toggle(button, fn)` | Same, but hands the `bool` to `fn` every cycle — for pneumatics or anything that isn't a motor. |
| `press(button, fn)` | Fires once on the rising edge. |
| `release(button, fn)` | Fires once on the falling edge. |
| `combo({A, B}, fn)` | Fires once when every button is held together, then rearms only after the whole combo is let go. |
| `macro(button, fn)` | Runs `fn` in its own `pros::Task`, so it can `pros::delay()` without stalling the loop. Presses during a run are ignored. |
| `on(fn)` | Escape hatch: raw callback every `update()`. |

`motor` is a `pros::AbstractMotor&`, so a single `Motor` and a `MotorGroup`
both work. `mv` is millivolts, clamped to ±12000. Going past that prints a warning to
the terminal and to line 7 of the brain screen rather than silently clamping,
since a voltage that far off is usually a typo. It is not an `assert` on
purpose: an assert aborts the program, and a dead robot mid-match is worse
than a clamped motor.

## State

Valid from the first `update()` onward:

| Query | Meaning |
| --- | --- |
| `held(button)` | Down right now |
| `pressed(button)` | Went down this cycle |
| `released(button)` | Came up this cycle |
| `axis(axis)` | Stick position, -127 to 127 |

`update()` snapshots all twelve buttons into a bitmask before running any
binding, which is what makes the edge-triggered bindings possible — polling
`get_digital` inline can't see a *change*. Call it exactly once per loop.

## Adding a mechanism

Motors get defined in `main.cpp` under `RobotDefinition`; the binding goes on
the chain in `opcontrol`. Nothing needs touching in `control.hpp` unless the
mechanism needs a binding shape that doesn't exist yet.

## Reading the source

Four bits of C++ syntax carry the whole class. Once they're clear, every
binding is three lines of ordinary code.

### `std::function<void()>` is a variable that holds a function

`int` holds a number, `std::function<void()>` holds *something callable that
takes nothing and returns nothing*. `std::function<void(bool)>` holds
something callable that takes a `bool`. That's all. It lets `press(UP, ...)`
accept a chunk of code the same way it would accept a number, stash it in the
`bindings` list, and call it later.

### `[](){ }` is a function with no name

Writing a whole named function just to hand it to `press()` is noise, so C++
lets you write one inline. These two are the same thing:

```cpp
void raiseArm() { clawPivot.move_voltage(12000); }
ctl.press(UP, raiseArm);

ctl.press(UP, [] { clawPivot.move_voltage(12000); });
```

The `[]` marks the start of one. `{ }` is its body.

### The `[...]` list says what the function is allowed to see

A lambda can't see the local variables around it unless you say so — that's
what goes in the brackets. Inside `bidir`:

```cpp
return on([this, &motor, p, voltage] {   // <-- capture list
    ...
});
```

- `this` — the `TsuControl` itself, so the body can call `held()`
- `&motor` — the motor **by reference**, i.e. the real motor, not a copy. It
  has to be a reference; you want to spin the actual hardware.
- `p`, `voltage` — copied, because they're just a button pair and a number

The capture matters because `on()` stores this function and calls it every
loop, long after `bidir()` returned. Anything captured by copy is safely
frozen; anything captured by reference had better outlive the robot, which
motors declared at the top of `main.cpp` do.

### `return *this` is what lets bindings chain

Every binding ends by returning the `TsuControl&` it was called on, so the
next `.bidir(...)` has something to attach to. That's the only reason this
works:

```cpp
ctl.bidir(intakeMotor, R)
   .bidir(winch, L);
```

Split it into separate `ctl.bidir(...);` statements if you find that clearer —
identical behavior.

### Why a bitmask for the button state

`update()` packs all twelve buttons into one `unsigned int`, one bit each,
into `now`, and keeps last cycle's copy in `was`. Edge detection is then a
single operation:

```cpp
now & ~was     // pressed this cycle: down now, up before
was & ~now     // released this cycle
```

This is the reason `press`, `toggle`, `combo`, and `macro` can exist at all.
Calling `get_digital` inline the way `opcontrol` used to can only tell you a
button *is* down, never that it just *changed*.
