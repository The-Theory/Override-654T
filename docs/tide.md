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