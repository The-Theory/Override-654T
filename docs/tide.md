# _Tide_ Walkthrough

A controller library for **VEX** by Theo Hallgren





---





## Intro

Tide is a library for **PROS** to easily handle controller bindings and the general workflow of controller input to robot output. 

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





# First Binds

A very common mechanism to start with is the _intake_, a usually spinning mechanism powered by a motor or two. 

In this case, our intake relies on a `pros::Motor`, though a `pros::MotorGroup` would work just as well. Assuming it's called `intakeMotor`, we can add the following line after `tide::Control ctl(controller)`:

```cpp
ctl.when(R1).hold(intakeMotor);
```

This line assigns the `R1` buttons to spin our intake forward. But holding `R1` when testing is annoying, so let's make it a toggle instead:

```cpp
ctl.when(R1).toggle(intakeMotor);
```

Now, whenever `R1` is pressed, the direction of our intake will change directions. However, when actually driving, our driver would probably appreciate being able to use `R1` and `R2` to change directions. We can do this as such:

```cpp
ctl.when(R).bidir(intakeMotor);
```

**Tide** understands `R` as representing the `R1` and `R2` pair, meaning we don't have to worry about any of that logic.





---





