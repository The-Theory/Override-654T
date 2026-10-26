#include "robot.hpp"
#include "tide/autotune.hpp"

using namespace tide::dir;
using tide::Wall;



const int TURN_TIMEOUT = 1000;	// [ms] safety cap; shorter ones cut turns off early on a weak battery

void x_auton() {
	chassis.setPose(0, -65, SOUTH);  // Placed flush on the audience wall, facing it
	winchPID.moveTo(195);
	doubleToggle();
	odom.wallReset(Wall::AUDIENCE);

	// Go to goal
	clawPivotUp();
	pros::delay(200);
	winchPID.moveTo(60);
	chassis.moveToPose(-24, -48, 90, 2000, {.forwards = false, .minSpeed=50}, false);
	pros::delay(150);
	claw.move_voltage(-12000);
	pros::delay(230);

	// Drive away from goal w/ claw up
	claw.move_voltage(0);
	winchPID.moveBy(50);
	chassis.moveToPoint(-5, -47, 1500);
	pickupMacroPart1();

	// To cuppin with claw ready
	chassis.turnToHeading(42, TURN_TIMEOUT, {}, false);
	odom.pushUntilStall(-6500);
	odom.driveDist(2);

	// Pickup cuppin
	winchPID.moveTo(0);
	pickupMacroPart2();
	claw.move_voltage(12000);
	winchPID.moveBy(400);
	
	// Turn out
	chassis.turnToHeading(45, TURN_TIMEOUT, {}, false);
	clawPivotUp();
	odom.driveDist(-3);
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

	// Ready to new cuppin
	chassis.moveToPose(0, -36, 0, 2000);
	winchPID.moveTo(0);
	pros::delay(500);
	tareClaw();
	pros::delay(500);
	chassis.moveToPoint(4, -47, 1500, {}, false);

	// To new cupping with claw
	pickupMacroPart1();
	chassis.turnToHeading(-40, TURN_TIMEOUT, {}, false);
	winchPID.moveTo(200);
	odom.pushUntilStall(-6500);
	odom.driveDist(2);

	// Pickup
	winchPID.moveTo(0);
	pickupMacroPart2();
	claw.move_voltage(12000);
	winchPID.moveBy(400);
}

void left_base_auton() {
	chassis.setPose(-65, 0, WEST);  // Placed flush on the red wall, facing it
	pros::delay(100);
	clawIntakeMacro();
	doubleToggle();
	odom.wallReset(Wall::RED);

	// Go to goal
	clawPivotUp();
	pros::delay(100);
	winchPID.moveTo(60);
	chassis.moveToPose(-48, -23, NORTH, 2000, {.forwards = false, .minSpeed=50}, false);
	pros::delay(150);
	claw.move_voltage(-12000);
	pros::delay(230);

	// Go to cuppin
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

	// Go to new cuppin
	pros::delay(400);
	chassis.moveToPoint(-30, -24, 1500, {}, false);
	chassis.turnToPoint(-47, -48, TURN_TIMEOUT, {.forwards = false}, false);
	chassis.moveToPoint(-43 , -43, 2000, {.forwards = false}, false);

	// Pickup new cuppin
	clawPivotUp();
	pros::delay(800);
	claw.move_velocity(12000);
	winchPID.moveTo(30);
	pros::delay(1000);
	winchPID.moveTo(700);
	claw.move_velocity(0);

	// Score new cuppin
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

void left_auton() {
	left_base_auton();

	// Toggle
}

void skills_auton() {
	left_base_auton();
	winchPID.moveTo(200);

	// Park
	chassis.turnToPoint(-5, -5, TURN_TIMEOUT, {.forwards=false}, false);
	chassis.moveToPoint(-5, -5, 1600, {.forwards=false}, false);

	// Get new
	chassis.moveToPoint(-10, -10, 800, {}, false );
	chassis.turnToPoint(-24, 32, 2000, {}, false);
	clawIntakeMacro();
	intakeMotor.move_voltage(12000);
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


////////////////////////////////////////////////////////////////
#pragma region Tuning //////////////////////////////////////////
////////////////////////////////////////////////////////////////
// Relay auto-tunes. Pick on the brain, run with LEFT in driver; results print on the brain and terminal.
// Starting settings: raise amplitude if it reports no oscillation, lower it if the swing is violent.

void tune_winch() {
	tide::autotune("winch", tide::mechanismPlant(winchPID), {.setpoint = 400, .amplitude = 4000, .maxDeviation = 200, .hysteresis = 1});
	winchPID.moveTo(0);
}

void tune_claw_pivot() {
	tide::autotune("claw pivot", tide::mechanismPlant(clawPivotPID), {.setpoint = -70, .amplitude = 3000, .maxDeviation = 60, .hysteresis = 1});
	clawPivotUp();
}

// Needs ~2 ft clear around the robot
void tune_turn() {
	tide::autotune("turn", tide::turnPlant(leftMotors, rightMotors, imu), {.setpoint = 0, .amplitude = 4000, .maxDeviation = 45});
}

// Needs ~1 ft clear in front and behind
void tune_drive() {
	tide::autotune("drive", tide::drivePlant(leftMotors, rightMotors, verticalEncoder, lemlib::Omniwheel::NEW_2),
		{.setpoint = 0, .amplitude = 3000, .maxDeviation = 12, .hysteresis = 0.25});
}

#pragma endregion
////////////////////////////////////////////////////////////////

// Brain buttons cycle through these; first is the default
const std::vector<tide::selector::Auton> autons = {
	{"Skills", skills_auton},
	{"X",      x_auton},
	{"Left",   left_auton},
	{"TUNE winch",      tune_winch},
	{"TUNE claw pivot", tune_claw_pivot},
	{"TUNE turn",       tune_turn},
	{"TUNE drive",      tune_drive},
};
