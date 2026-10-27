#include "robot.hpp"

////////////////////////////////////////////////////////////////
#pragma region Macros //////////////////////////////////////////
////////////////////////////////////////////////////////////////

const double TOGGLE_BACKOFF = 4;	// [in]
const int    TOGGLE_HOLD    = 250;	// [ms]

// From scoring height; ends with the winch down
void score() {
	outtakeClaw();
	pros::delay(300);
	winchPID.moveBy(200);		// Lift off the stack
	pros::delay(500);			// Time for the driver to back away
	stopClaw();
	winchPID.moveTo(0);
}

// Two hits on the Toggle in front of the robot
void doubleToggle() {
	for (int i = 0; i < 2; i++) {
		odom.moveBy(-TOGGLE_BACKOFF);
		odom.pushUntilStall(6500, 1500, TOGGLE_HOLD);
	}
}

// Claw control
void intakeClaw() 	{ claw.move_voltage(12000); }
void stopClaw() 	{ claw.move_voltage(0); 	}
void outtakeClaw() 	{ claw.move_voltage(-12000);}

#pragma endregion
////////////////////////////////////////////////////////////////
