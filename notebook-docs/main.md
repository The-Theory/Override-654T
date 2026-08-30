# `src/main.cpp` - Walkthrough

This document explains what the main `src/main.cpp` achieves for our robot. Each section is in the same order as the file, top to bottom. Our code is built on two main software libraries: **PROS** (an operating system for the Brain) and **LemLib** (a driving and odometry library).

---

## File header

```cpp
////////////////////////////////////////////////////////////////
// Codebase of 654T - Tsunami                                 //
// Lake Travis High School, Texas, United States              //
// VEX Override 2026-2027                                     //
//                                                            //
// By Theo Hallgren, Ryan Koontz, and Sebastian Ditsch        //
// Using LemLib by Liam Teale                                 //
//                                                            //
// Source available on: github.com/The-Theory/Override-654T   //
// under GPL-3.0 license                                      //
////////////////////////////////////////////////////////////////
```

This comment block at the very top credits the coding team and gives general information about our code's purpose and foundation. This is documentation only, having no effect on how the robot.

---

## Library imports

```cpp
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
```

These imports pull-in outside code so this file can use it. The note `IWYU pragma: keep` on the **LemLib** line is an instruction to code-cleanup tools telling them not delete that import automatically.

---

## Hardware ports and objects

```cpp
const int UNDEF_PORT = 0;
pros::MotorGroup leftMotors ({-7, -9}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({ 8, 10}, pros::MotorGearset::blue);
pros::Rotation vertical_encoder(UNDEF_PORT);
pros::Imu imu(UNDEF_PORT);
pros::Motor clawPivot(4, pros::MotorGearset::green);

pros::Controller controller(pros::E_CONTROLLER_MASTER);
```

Here we define all of our ports, and define the controller object. Reverse numbers mean reversed motors, so, for example,the left side of our drivetrain uses **Ports** `7` and `9`, with both motors reversed. We also specify the cartridge for each motor. In code, a **5.5W** motor is handled the same as a green **11W** motor. 

Both our vertical encoder (rotation sensor for forward and backwards motion) and our **IMU** (Inertial Measurement Unit)haven't been physically attached yet, so we included a temporary assignement of "**Port** `0`."

---

## Input curves

```cpp
lemlib::ExpoDriveCurve throttle_curve(
    3,      // joystick deadband out of 127
    10,     // minimum output where drivetrain will move out of 127
    1.019   // expo curve gain
);
lemlib::ExpoDriveCurve steer_curve(
    3,      // joystick deadband out of 127
    10,     // minimum output where drivetrain will move out of 127
    1.019   // expo curve gain
);
```

This code defines our driving sensitivities. These values are only placeholders suggested by the LemLib documentation, but seem to work quite well. Common issues like oversensitive turning or stick-drift can be mitigated by altering these settings. We'll continue to edit these throughout the year, seeing what our driver prefers.

---

## Drivetrain physical specs

```cpp
lemlib::Drivetrain drivetrain(
    &leftMotors,                // left
    &rightMotors,               // right
    11.4173,                    // track width
    lemlib::Omniwheel::NEW_275, // wheel type
    450,                        // drivetrain rpm
    2                           // horizontal drift is 2 (for now)
);
```

This portion defines our drivetrain, using a few measured parameters. Track width is the width between the left and right side of the drivetrain, measured in inches from the wheels. We also tell **LemLib** that we are using **2.75"** wheels, and that our drivetrain outputs **450 RPM**. The final `Horizontal Drift` parameter, currently set to **2**, is used to offset any drift our drivetrain might have. This will be properly calibrated when we finish building this iteration, as it may shift with weight changes.

---

## Odometry (position tracking) 

```cpp
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel,
    nullptr, nullptr, nullptr,  // unused tracking wheels
    &imu
);
```

Here we give LemLib our odometry information, where we have one **2"** tracking wheel for vertical movement, and an **IMU**. **LemLib** supports the use of several tracking wheels, but we are only using one. 

---

## PID tuning - driving (lateral controller)

```cpp
lemlib::ControllerSettings lateral_controller(
    10, // prop gain      (kP)
    0,  // integral gain   (kI)
    3,  // derivative gain (kD)
    3,  // anti windup
    1,  // small error range           [in]
    100,// small error range timeout   [ms]
    3,  // large error range           [in]
    500,// large error range timeout   [ms]
    20  // maximum acceleration (slew)
);
```

**"PID"** is the self-correcting math **LemLib** uses during autonomous to drive to a target distance and stop cleanly instead of overshooting or oscillating. This code tunes it for forward movement. Similar to our `Horizontal Drift` parameter, these values will be calibrated once we finish building.

---

## PID tuning — turning (angular controller)

```cpp
lemlib::ControllerSettings angular_controller(
    2,  // prop gain       (kP)
    0,  // integral gain   (kI)
    10, // derivative gain (kD)
    3,  // anti windup
    1,  // small error range           [deg]
    100,// small error range timeout   [ms]
    3,  // large error range           [deg]
    500,// large error range timeout   [ms]
    0   // maximum acceleration (slew)
);
```

Same idea as the previous section, just that here we handle turning-based movements. Will also be calibrated properly in the future.

---

## Chassis assembly

```cpp
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);
```

This one line combines every section above into one object: the **Chassis**. This is what **LemLib** will use to handle our instructions into physical changes on the robot. 

---

## Screen button callback (`on_center_button`)

```cpp
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) pros::lcd::set_text(2, "I was pressed!");
    else pros::lcd::clear_line(2);
}
```

A small helper function that runs each time someone presses the center button on the Brain's screen. At the moment, it just toggles a text on-screen. This is an example function from the **LemLib** docs, and will be used in the future to handle **Automation Routine Selection**.

---

## Startup

```cpp
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");
    pros::lcd::register_btn1_cb(on_center_button);
}
```

Runs once, automatically, the moment the robot powers on. Currently it only turns on
the Brain's text display, prints "Hello PROS User!" on the first line, and connects
the center screen button to the callback above. All three are default template
actions — no sensor calibration, no autonomous selector has been added, so this is
effectively still a stub.

---

## Disabled period (`disabled`)

```cpp
void disabled() {}
```

Runs whenever the field control system disables the robot: before the match, between
the autonomous and driver phases, and after the match. It is intentionally **empty**.
During a disabled period the robot is supposed to do nothing, so an empty body is the
correct and complete implementation — not a missing piece.

---

## Autonomous selector (`competition_initialize`)

```cpp
void competition_initialize() {}
```

Runs after startup but before the match begins — the normal place to build a menu for
choosing which autonomous routine to run (for example, by starting position). It is
currently **empty**, a deliberate stub. There is nothing to select yet because no
autonomous routines have been written.

---

## Autonomous period (`autonomous`)

```cpp
void autonomous() {}
```

The roughly 15-second phase at the start of a match where the robot runs a
pre-programmed routine with no driver input. It is currently **empty**, so the robot
sits still for the entire autonomous period. This is the biggest unfinished piece of
the program. The driving and PID machinery it would rely on is already set up; the
routine itself has not been written.

---

## Driver control loop (`opcontrol`)

```cpp
void opcontrol() {
    while (true) {
        pros::delay(25);

        // Get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Dual-stick arcade
        chassis.curvature(leftY, rightX);

        // Claw control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            clawPivot.move_voltage(12000);
    }
}
```

The driver-controlled phase, and the only behavior section that is actually
implemented. It runs a loop that repeats until the phase ends. Each pass:

1. **Wait 25 milliseconds.** Paces the loop to about 40 cycles per second. Without a
   pause it would run thousands of times per second and needlessly load the Brain's
   processor. The wait is at the top so it happens every cycle regardless of which
   branches run.
2. **Read two joystick axes:** left stick up/down (`leftY`), right stick left/right
   (`rightX`).
3. **Drive using `chassis.curvature`** (dual-stick arcade). `leftY` sets
   forward/backward speed; `rightX` sets how sharply the robot curves. This makes the
   robot handle like a car — at higher speed the same stick input gives a wider turn
   — which many drivers find smoother than tank controls. Design note: the raw
   joystick values go straight in. The input curves defined earlier are **not**
   applied here — wiring them in is an obvious next step.
4. **Claw control:** while **R1** is held, the claw motor runs at full power (12000
   millivolts = 12 volts). There is no command to run it the other way or actively
   hold it, so releasing R1 just stops driving the motor and it coasts. A
   reverse/retract button is a known missing feature.

---

## Current state summary

**Working now**

- Full hardware definition for the four-motor drivetrain and the claw motor.
- Driver control: dual-stick arcade driving plus a one-direction claw button (R1).
- Startup screen text and the template test button.

**Defined but not yet functional**

- Odometry / position tracking — the tracking-wheel sensor and IMU have no ports.
- PID tuning values — present, but only used by autonomous, which is empty.
- Input curves — created, but not applied to live driving.

**Not started**

- Autonomous routine (empty stub).
- Autonomous selector (empty stub).
- Claw retract / reverse control.

**Placeholder values to revisit**

- Horizontal drift (2) — explicitly marked "for now."
- Sensor ports set to `UNDEF_PORT` (0) — undefined.
- All PID gains — first-pass estimates, untested on the real robot.

