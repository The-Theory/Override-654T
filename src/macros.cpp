#include "robot.hpp"

////////////////////////////////////////////////////////////////
#pragma region Macros //////////////////////////////////////////
////////////////////////////////////////////////////////////////

const double TOGGLE_BACKOFF = 4;	// [in]

// From scoring height; ends with the winch down
void score() {
	claw.move_voltage(-12000);	// Spit out
	pros::delay(300);
	winchPID.moveBy(200);		// Lift off the stack
	pros::delay(500);			// Time for the driver to back away
	claw.move_voltage(0);
	winchPID.moveTo(0);
}

// Two hits on the Toggle in front of the robot
void doubleToggle() {
	for (int i = 0; i < 2; i++) {
		odom.moveBy(-TOGGLE_BACKOFF);
		odom.pushUntilStall(6500);
	}
}

#pragma endregion
////////////////////////////////////////////////////////////////
