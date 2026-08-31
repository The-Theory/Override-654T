# `src/main.cpp` - Walkthrough

This document explains what the main `src/main.cpp` achieves for our robot. Each section is in the same order as the file, top to bottom. Our code is built on two main software libraries: **PROS** (an operating system for the **Brain**) and **LemLib** (a driving and odometry library).
 
---

## File Header

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

## Library Imports

```cpp
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
```

These imports pull-in outside code so this file can use it. The note `IWYU pragma: keep` on the **LemLib** line is an instruction to code-cleanup tools telling them not delete that import automatically.

---

## Hardware Ports and Objects

```cpp
const int UNDEF_PORT = 0;
pros::MotorGroup leftMotors ({-7, -9}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({ 8, 10}, pros::MotorGearset::blue);
pros::Rotation vertical_encoder(UNDEF_PORT);
pros::Imu imu(UNDEF_PORT);
pros::Motor clawPivot(4, pros::MotorGearset::green);

pros::Controller controller(pros::E_CONTROLLER_MASTER);
```

Here we define all of our ports, and define the controller object. Reverse numbers mean reversed motors, so, for example, the left side of our drivetrain uses **Ports** `7` and `9`, with both motors reversed. We also specify the cartridge for each motor. In code, a **5.5W** motor is handled the same as a green **11W** motor. 

Both our vertical encoder (rotation sensor for forward and backwards motion) and our **IMU** (Inertial Measurement Unit) haven't been physically attached yet, so we included a temporary assignment of an imaginary "**Port** `0`."

We also define our **Controller** to be used during operator control.

---

## Input Curves

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

## Drivetrain

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

## PID - Driving (lateral controller)

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

## PID - Turning (angular controller)

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

Same idea as the previous section, just that here we handle turning-based movements. This will also be calibrated properly in the future.

---

## Chassis Assembly

```cpp
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);
```

This one line combines every section above into one object: the **Chassis**. This is what **LemLib** will use to handle our instructions into physical changes on the robot. 

---

## Screen Button Callback

```cpp
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) pros::lcd::set_text(2, "I was pressed!");
    else pros::lcd::clear_line(2);
}
```

A small helper function that runs each time someone presses the center button on the Brain's screen. At the moment, it just toggles a text on-screen. This is an example function from the **LemLib** docs, and will be used as a template in the future to handle **Automation Routine Selection**.

---

## Startup

```cpp
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");
    pros::lcd::register_btn1_cb(on_center_button);
}
```

This section is also unchanged from the default **LemLib** configuration. The `initialize()` function is run automatically whenever the program is started. Currently, it just displays some text on the screen, and adds the button shown in the section above. It won't really be used to anything, as it's used mostly by **LemLib** to calibrate sensors and get the system up and running.

---

## Disabled Period

```cpp
void disabled() {}
```

This function is also called automatically, but is instead run whenever the robot is commanded to go into a **disabled** state. This is usually done before a match starts, or during the intermission time between the **Autonomous Control Period** and the **Driver Controller Period**. It will stay empty, as a robot cannot move during its disabled period.

---

## Autonomous Selector 

```cpp
void competition_initialize() {}
```

This function is run by **LemLib** after startup but before a match actually starts. Thus, this is what will be used to select autonomous routines in the future. Since the code is still developing as the robot is being built, we do not currently have any autonomous routines defined, but we have several ones planned. Furthermore, once we acquire our own field, autonomous development will ramp up heavily.

---

## Autonomous Period

```cpp
void autonomous() {}
```

This is the function that will be running during the **15-second Autonomous Control Period** at the start of each match. We'll also be calling this function manually during autonomous development. **LemLib** uses the sensors we defined previously to help track its position on the field, and does so automatically. 

---

## Driver Control 

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

This section is the only part of the code that actually defines behavior so far. Let's see what it does, line by line:

- The `delay` function slows down the program to around 40 cycles per second, saving the CPU from trying to run the program as fast as possible, which is pointless. Note that speed and fluidity of motion is not affected by cycle frequency. 
<br>

- The two `get_analog` lines request values from the **Controller** and save them. Specifically, we ask for the vertical position of the left stick, and the horizontal position of the right. 
<br>

- The `curvature` function is called using our **Chassis** object, instructing **LemLib** to use the **Controllers** saved values to move the robot using curvature drive. 
<br>

- Finally, the `get_digital` function asks the **Controller** if the **R1** button is being pressed, and if so, rotates the `clawPivot` motor with 1200mV. This just means that when the **R1** button is pressed, we spin the claw motor at max speed forward. Currently, this is just testing, but is the precursor for all of our future controls. 

---

## Current State

- [x] Driver code
- [x] Motor ports
- [x] Full **LemLib** support
- [x] GitHub repo 
- [x] Full codebase documentation
- [ ] **PID** Tuning
- [ ] Stage 1 autonomous
- [ ] Full claw control
- [ ] Horizontal drift tuning
- [ ] Sensor ports

---
# -TH