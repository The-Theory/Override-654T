#pragma once

#include "robot.hpp"

constexpr double TOGGLE_BACKOFF = 4;	// [in]
constexpr int    TOGGLE_HOLD    = 250;	// [ms]

// Claw control
inline void intakeClaw() 	{ claw.move_voltage(12000); }
inline void stopClaw() 	{ claw.move_voltage(0); 	}
inline void outtakeClaw() 	{ claw.move_voltage(-12000);}

// From scoring height; ends with the winch down
inline void score() {
	outtakeClaw();
	pros::delay(300);
	winch.moveBy(200);		// Lift off the stack
	pros::delay(500);			// Time for the driver to back away
	stopClaw();
	winch.moveTo(0);
}

// Two hits on the Toggle in front of the robot
inline void doubleToggle() {
	for (int i = 0; i < 2; i++) {
		odom.moveBy(-TOGGLE_BACKOFF);
		odom.pushUntilStall(6500, 1500, TOGGLE_HOLD);
	}
}
