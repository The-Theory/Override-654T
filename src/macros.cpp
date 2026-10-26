#include "robot.hpp"



////////////////////////////////////////////////////////////////
#pragma region Checks //////////////////////////////////////////
////////////////////////////////////////////////////////////////

bool isWinchDown() { return winchEncoder.get_position() < 480; }

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Macros //////////////////////////////////////////
////////////////////////////////////////////////////////////////

const int CLAW_PIVOT_UP   = -140;	// [deg]
const int CLAW_PIVOT_DOWN = 0;		// [deg]
const double TOGGLE_BACKOFF = 4;	// [in]

bool clawDropped = false;

void clawPivotUp() {
	clawPivotPID.moveTo(CLAW_PIVOT_UP);
	clawDropped = false;
}
void clawPivotDown() { clawPivotPID.moveTo(CLAW_PIVOT_DOWN); }

// From scoring height with the claw up; ends with the winch down and the claw dropped
void score() {
	claw.move_voltage(-12000);	// Spit out
	pros::delay(300);
	winchPID.moveBy(200);		// Lift off the stack
	clawPivotPID.moveBy(-100);
	pros::delay(500);			// Time for the driver to back away
	claw.move_voltage(0);
	winchPID.moveTo(0);
	clawPivotPID.moveTo(CLAW_PIVOT_DOWN);
	clawDropped = true;

	while (!isWinchDown())
		pros::delay(100);
	clawPivotPID.release();
}

// Tilts the claw past up so it clears the pin before coming down on it
void raiseForPickup() { clawPivotPID.moveTo(CLAW_PIVOT_UP - 75); }

// Grabs a floor pin; ends with it held and the claw up
void pickup(int time) {
	claw.move_voltage(12000);
	clawPivotDown();
	pros::delay(time);
	clawPivotUp();
	claw.move_voltage(0);
}

// Back to intake height after score() dropped the claw
void lowerToIntake() {
	if (clawDropped) {
		winchPID.moveTo(195);
		clawPivotPID.moveTo(50);
	}
}

// Lets the pivot fall to rest and zeroes there
void tareClawPivot() {
	clawPivotPID.release();
	pros::delay(400);
	clawPivotPID.tare();
	clawPivotPID.start();
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
