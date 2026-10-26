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

// Defs
const int CLAW_PIVOT_UP   = -140;	// [deg]
const int CLAW_PIVOT_DOWN = 0;	// [deg]
bool downState = false;

void clawPivotUp()   {
	clawPivotPID.moveTo(CLAW_PIVOT_UP);
	downState = false;
}
void clawPivotDown() { clawPivotPID.moveTo(CLAW_PIVOT_DOWN); }

/**
 * Run when needing to score.
 * Start: Cascade level to score, claw in Up position.
 * End: Intake position; Cascade down, claw down.
 */
void scoringMacro() {
	claw.move_voltage(-12000);  // Spit out from claw
	pros::delay(300);
	winchPID.moveBy(200); 		// Lift cascade a tad
	clawPivotPID.moveBy(-100);  // Move claw up slightly
	pros::delay(500);  			// Wait for OP drive back
	claw.move_voltage(0);  		// Stop claw
	winchPID.moveTo(0);  		// Move winch down
	clawPivotPID.moveTo(CLAW_PIVOT_DOWN); // Move claw down
	downState = true;

	while (!isWinchDown())
		pros::delay(100);
	clawPivotPID.release();		// Drop claw
}

/**
 * Run when needing to pick a pin off of the floor.
 * Start: Cascade level to pick up, claw in up position
 * End: Same; pin in claw.
 */
void pickupMacroPart1() {
	clawPivotPID.moveTo(CLAW_PIVOT_UP - 75);  // Lift claw
}
void pickupMacroPart2(int pickUpTime) {
	claw.move_voltage(12000);	// Spin claw
	clawPivotDown();
	pros::delay(pickUpTime);
	clawPivotUp();				// Make claw horizontal
	claw.move_voltage(0);		// Stop claw
}

 /**
 * Run when needing to pick a pin off of the floor.
 * Start: Cascade level to pick up, claw in up position
 * End: Same; pin in claw.
 */
void clawIntakeMacro() {
	if(downState) {
		winchPID.moveTo(195);
		clawPivotPID.moveTo(50);
	}

}

void tareClaw() {
	clawPivotPID.release();
	pros::delay(400);
	clawPivotPID.tare();
	clawPivotPID.start();
}

const double TOGGLE_BACKOFF = 4;	// [in] tune: was a 200 ms timed back-up

// Two hits on the Toggle in front of the robot
void doubleToggle() {
	for (int i = 0; i < 2; i++) {
		odom.driveDist(-TOGGLE_BACKOFF);
		odom.pushUntilStall(6500);
	}
}

#pragma endregion
////////////////////////////////////////////////////////////////
