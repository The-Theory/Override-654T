#include "robot.hpp"
#include "tide/autotune.hpp"

using namespace tide::dir;
using tide::Wall;

const int TURN_TIMEOUT = 1000;	// [ms] long enough to finish on weak battery

////////////////////////////////////////////////////////////////
#pragma region Routines ////////////////////////////////////////
////////////////////////////////////////////////////////////////

void xAuton(tide::Alliance) {
	chassis.setPose(0, -65, SOUTH);	// Flush on the audience wall, facing it
	winchPID.moveTo(195);
	doubleToggle();
	odom.wallReset(Wall::AUDIENCE);

	// To goal
	clawPivotUp();
	pros::delay(200);
	winchPID.moveTo(60);
	chassis.moveToPose(-24, -48, 90, 2000, {.forwards = false, .minSpeed=50}, false);
	pros::delay(150);
	claw.move_voltage(-12000);
	pros::delay(230);

	// Back off the goal, claw up
	claw.move_voltage(0);
	winchPID.moveBy(50);
	chassis.moveToPoint(-5, -47, 1500);
	raiseForPickup();

	// To cuppin
	chassis.turnToHeading(42, TURN_TIMEOUT, {}, false);
	odom.pushUntilStall(-6500);
	odom.moveBy(2);

	// Pickup cuppin
	winchPID.moveTo(0);
	pickup();
	claw.move_voltage(12000);
	winchPID.moveBy(400);
	
	// Turn out
	chassis.turnToHeading(45, TURN_TIMEOUT, {}, false);
	clawPivotUp();
	odom.moveBy(-3);
	chassis.turnToHeading(55, TURN_TIMEOUT, {}, false);
	chassis.turnToHeading(EAST, TURN_TIMEOUT, {}, false);
	odom.wallReset(Wall::AUDIENCE);
	chassis.moveToPose(-12, -60, EAST, 1500, {}, false);
	clawPivotUp();
	chassis.turnToHeading(SOUTH_EAST, TURN_TIMEOUT, {}, false);
	chassis.moveToPose(-26, -45, SOUTH_EAST, 1100, {.forwards = false}, false);

	// Score
	claw.move_voltage(0);
	winchPID.moveTo(100);
	pros::delay(500);
	clawPivotPID.release();
	pros::delay(200);
	claw.move_voltage(-12000);
	pros::delay(500);
	winchPID.moveTo(800);
	pros::delay(500);
	claw.move_voltage(0);

	// Reset for next cuppin
	chassis.moveToPose(0, -36, 0, 2000);
	winchPID.moveTo(0);
	pros::delay(500);
	tareClawPivot();
	pros::delay(500);
	chassis.moveToPoint(4, -47, 1500, {}, false);

	// To next cuppin
	raiseForPickup();
	chassis.turnToHeading(-40, TURN_TIMEOUT, {}, false);
	winchPID.moveTo(200);
	odom.pushUntilStall(-6500);
	odom.moveBy(2);

	// Pickup
	winchPID.moveTo(0);
	pickup();
	claw.move_voltage(12000);
	winchPID.moveBy(400);
}

void leftBase() {
	chassis.setPose(-65, 0, WEST);	// Flush on the red wall, facing it
	pros::delay(100);
	lowerToIntake();
	doubleToggle();
	odom.wallReset(Wall::RED);

	// To goal
	clawPivotUp();
	pros::delay(100);
	winchPID.moveTo(60);
	chassis.moveToPose(-48, -23, NORTH, 2000, {.forwards = false, .minSpeed=50}, false);
	pros::delay(150);
	claw.move_voltage(-12000);
	pros::delay(230);

	// To cuppin
	winchPID.moveTo(400);
	chassis.moveToPose(-48, -10, NORTH, 1000, {.minSpeed=50});
	chassis.moveToPose(-24 - 2.2, -24 + 2.2, NORTH_WEST, 1500, {.forwards = false, .minSpeed=40}, false);

	// Pickup cuppin
	clawPivotUp();
	claw.move_velocity(12000);
	pros::delay(100);
	winchPID.moveTo(20);
	pros::delay(1500);
	winchPID.moveTo(600);
	claw.move_velocity(0);

	// Score cuppin
	chassis.turnToPoint(48, 24, TURN_TIMEOUT, {}, false);
	chassis.moveToPoint(-48 + 3, -24, 600, {.forwards = false, .minSpeed=50}, false);
	winchPID.moveTo(15);
	pros::delay(400);
	claw.move_velocity(-12000);
	pros::delay(400);
	claw.move_velocity(0);
	winchPID.moveTo(600);

	// To next cuppin
	pros::delay(400);
	chassis.moveToPoint(-30, -24, 1500, {}, false);
	chassis.turnToPoint(-47, -48, TURN_TIMEOUT, {.forwards = false}, false);
	chassis.moveToPoint(-43, -43, 2000, {.forwards = false}, false);

	// Pickup
	clawPivotUp();
	pros::delay(800);
	claw.move_velocity(12000);
	winchPID.moveTo(30);
	pros::delay(1000);
	winchPID.moveTo(700);
	claw.move_velocity(0);

	// Score
	chassis.turnToPoint(-48, -24, TURN_TIMEOUT, {.forwards = false}, false);
	chassis.moveToPoint(-48, -24 - 3, 800, {.forwards = false, .maxSpeed=100}, false);
	winchPID.moveTo(15);
	pros::delay(400);
	claw.move_velocity(-12000);
	pros::delay(800);
	claw.move_velocity(0);
	winchPID.moveTo(1000);
	pros::delay(500);

	// Back out
	chassis.moveToPoint(-48, -48, 1500, {}, false);
}

void leftAuton(tide::Alliance) {
	leftBase();
	// TODO: Toggle
}

void skillsAuton(tide::Alliance) {
	leftBase();
	winchPID.moveTo(200);

	// Park
	chassis.turnToPoint(-5, -5, TURN_TIMEOUT, {.forwards=false}, false);
	chassis.moveToPoint(-5, -5, 1600, {.forwards=false}, false);

	// Grab from midfield
	chassis.moveToPoint(-10, -10, 800, {}, false);
	chassis.turnToPoint(-24, 32, 2000, {}, false);
	lowerToIntake();
	intake.move_voltage(12000);
	claw.move_voltage(12000);
	chassis.moveToPose(-24, 32, NORTH, 2000, {}, false);
	pros::delay(4000);

	// Score
	clawPivotUp();
	winchPID.moveTo(250);
	claw.move_voltage(0);
	chassis.moveToPoint(0, 0, 2000, {.forwards=false}, false);
	claw.move_voltage(-12000);
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Tuning //////////////////////////////////////////
////////////////////////////////////////////////////////////////

std::string tuneWinch() {
	const tide::TuneResult result = tide::autotune("winch", tide::mechanismPlant(winchPID), {.setpoint = 400, .amplitude = 4000, .maxDeviation = 200, .hysteresis = 1});
	winchPID.moveTo(0);
	return result.summary;
}

std::string tuneClawPivot() {
	const tide::TuneResult result = tide::autotune("claw pivot", tide::mechanismPlant(clawPivotPID), {.setpoint = -70, .amplitude = 3000, .maxDeviation = 60, .hysteresis = 1});
	clawPivotUp();
	return result.summary;
}

// Needs ~2 ft clear around the robot
std::string tuneTurn() {
	return tide::autotune("turn", tide::turnPlant(leftMotors, rightMotors, imu), {.setpoint = 0, .amplitude = 4000, .maxDeviation = 45}).summary;
}

// Needs ~1 ft clear in front and behind
std::string tuneDrive() {
	return tide::autotune("drive", tide::drivePlant(leftMotors, rightMotors, verticalEncoder, lemlib::Omniwheel::NEW_2),
		{.setpoint = 0, .amplitude = 3000, .maxDeviation = 12, .hysteresis = 0.25}).summary;
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region SelectorScreen //////////////////////////////////
////////////////////////////////////////////////////////////////

// Auton page
const std::vector<tide::screen::Auton> autons = {
	{"X",      xAuton},			// Audience wall, centered on the Toggle, facing it
	{"Left",   leftAuton},		// Alliance wall, centered on the Toggle, facing it
	{"Skills", skillsAuton},	// Skills only - crosses Auton Line
};

// Tune page
const std::vector<tide::screen::Tuner> tuners = {
	{"Winch",      tuneWinch},
	{"Claw pivot", tuneClawPivot},
	{"Turn",       tuneTurn},
	{"Drive",      tuneDrive},
};

#pragma endregion
////////////////////////////////////////////////////////////////
