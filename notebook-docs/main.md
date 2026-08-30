# `src/main.cpp` — Walkthrough

This document explains what the main `src/main.cpp` achieves for our robot. Each section is in the same order as the file, top to bottom. Our code is built on two main software libraries: PROS (an operating system for the Brain) and LemLib (a driving and odometry library).

---

## File header

```cpp
////////////////////////////////////////////////////////////////
// Codebase of 654T - Tsunami                                  //
// Lake Travis High School, Texas, United States               //
// VEX Override 2026-2027                                       //
//                                                             //
// By Theo Hallgren, Ryan Koontz, and Sebastian Ditsch         //
// Using LemLib by Liam Teale                                   //
//                                                             //
// Source available on: github.com/The-Theory/Override-654T    //
// under GPL-3.0 license                                        //
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

Both our vertical encoder (rotation sensor for forward and backwards motion) and our IMU (Inertial Measurement Unit)haven't been physically attached yet, so we included a temporary assignement of "**Port** `0`."

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
