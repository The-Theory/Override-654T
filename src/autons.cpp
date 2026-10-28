#include "macros.hpp"
#include "tide/autotune.hpp"
#include "tide/override.hpp"
#include "tide/screen.hpp"
#include <algorithm>
#include <cstdio>

using namespace tide::dir;
using namespace tide::override;
using tide::Wall;

////////////////////////////////////////////////////////////////
#pragma region Routines ////////////////////////////////////////
////////////////////////////////////////////////////////////////

const int TURN_TIMEOUT = 1000;	// [ms] long enough to finish on weak battery

void xAuton(tide::Alliance) {
	chassis.setPose(0, -65, SOUTH);	// Flush on the audience wall facing toggle
	doubleToggle();
	odom.wallReset(Wall::AUDIENCE);

	// To goal
	odom.reachPose(goal::Q2_ALLIANCE, EAST, 2000, {.forwards = false, .minSpeed = 50});
	pros::delay(150);
	outtakeClaw();
	pros::delay(230);

	// Back off the goal, claw up
	stopClaw();
	winch.moveBy(50);
	chassis.moveToPoint(-5, -47, 1500);

	// To cuppin
	odom.aim(cuppin::AUDIENCE_WALL_W, TURN_TIMEOUT);
	odom.pushUntilStall(-6500);
	odom.moveBy(2);

	// Pickup cuppin
	winch.moveTo(0);
	intakeClaw();
	winch.moveBy(400);
	
	// Turn out
	chassis.turnToHeading(45, TURN_TIMEOUT, {}, false);
	odom.moveBy(-3);
	chassis.turnToHeading(55, TURN_TIMEOUT, {}, false);
	chassis.turnToHeading(EAST, TURN_TIMEOUT, {}, false);
	odom.wallReset(Wall::AUDIENCE);
	chassis.moveToPose(-12, -60, EAST, 1500, {}, false);
	chassis.turnToHeading(SOUTH_EAST, TURN_TIMEOUT, {}, false);
	odom.reachPose(goal::Q2_ALLIANCE, SOUTH_EAST, 1100, {.forwards = false});

	// Score
	stopClaw();
	winch.moveTo(100);
	pros::delay(500);
	pros::delay(200);
	outtakeClaw();
	pros::delay(500);
	winch.moveTo(800);
	pros::delay(500);
	stopClaw();

	// Reset for next cuppin
	chassis.moveToPose(0, -36, 0, 2000);
	winch.moveTo(0);
	pros::delay(500);
	pros::delay(500);
	chassis.moveToPoint(4, -47, 1500, {}, false);

	// To next cuppin
	odom.aim(cuppin::AUDIENCE_WALL_E, TURN_TIMEOUT);
	winch.moveTo(200);
	odom.pushUntilStall(-6500);
	odom.moveBy(2);

	// Pickup
	winch.moveTo(0);
	intakeClaw();
	winch.moveBy(400);
}

void leftBase() {
	chassis.setPose(-65, 0, WEST);	// Flush on the red wall, facing it
	pros::delay(100);
	doubleToggle();
	odom.wallReset(Wall::RED);

	// To goal
	pros::delay(100);
	winch.moveTo(60);
	odom.reachPose(goal::Q1_ALLIANCE, NORTH, 2000, {.forwards = false, .minSpeed = 50});
	pros::delay(150);
	outtakeClaw();
	pros::delay(230);

	// To cuppin
	winch.moveTo(400);
	chassis.moveToPose(-48, -10, NORTH, 1000, {.minSpeed=50});
	odom.reachPose(cuppin::DIAGONAL_INNER, NORTH_WEST, 1500, {.forwards = false, .minSpeed = 40});

	// Pickup cuppin
	intakeClaw();
	pros::delay(100);
	winch.moveTo(20);
	pros::delay(1500);
	winch.moveTo(600);
	stopClaw();

	// Score cuppin
	odom.aim(goal::Q1_ALLIANCE, TURN_TIMEOUT);
	odom.reachPoint(goal::Q1_ALLIANCE, 600, {.minSpeed = 50});
	winch.moveTo(15);
	pros::delay(400);
	outtakeClaw();
	pros::delay(400);
	stopClaw();
	winch.moveTo(600);

	// To next cuppin
	pros::delay(400);
	chassis.moveToPoint(-30, -24, 1500, {}, false);
	odom.aim(cuppin::DIAGONAL_OUTER, TURN_TIMEOUT);
	odom.reachPoint(cuppin::DIAGONAL_OUTER);

	// Pickup
	pros::delay(800);
	intakeClaw();
	winch.moveTo(30);
	pros::delay(1000);
	winch.moveTo(700);
	stopClaw();

	// Score
	odom.aim(goal::Q1_ALLIANCE, TURN_TIMEOUT);
	odom.reachPoint(goal::Q1_ALLIANCE, 800, {.maxSpeed = 100});
	winch.moveTo(15);
	pros::delay(400);
	outtakeClaw();
	pros::delay(800);
	stopClaw();
	winch.moveTo(1000);
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
	winch.moveTo(200);

	// Park
	chassis.turnToPoint(-5, -5, TURN_TIMEOUT, {.forwards=false}, false);
	chassis.moveToPoint(-5, -5, 1600, {.forwards=false}, false);

	// Grab from midfield
	chassis.moveToPoint(-10, -10, 800, {}, false);
	chassis.turnToPoint(-24, 32, 2000, {}, false);
	intakeClaw();
	chassis.moveToPose(-24, 32, NORTH, 2000, {}, false);
	pros::delay(4000);

	// Score
	winch.moveTo(250);
	stopClaw();
	odom.reachPoint(goal::MIDFIELD);
	outtakeClaw();
}

void testAuton(tide::Alliance) {

}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Tuning //////////////////////////////////////////
////////////////////////////////////////////////////////////////

std::string tuneWinch() {
	const tide::TuneResult result = tide::autotune("winch", tide::mechanismPlant(winch), 
		{.setpoint = 400, .amplitude = 4000, .maxDeviation = 200, .hysteresis = 1});
	winch.moveTo(0);
	return result.summary;
}

// Needs ~2 ft clear around the robot
std::string tuneAngular() {
	return tide::autotune("angular", tide::angularPlant(leftMotors, rightMotors, imu), 
		{.setpoint = 0, .amplitude = 4000, .maxDeviation = 45}).summary;
}

// Needs ~1 ft clear in front and behind
std::string tuneLateral() {
	return tide::autotune("lateral", tide::lateralPlant(leftMotors, rightMotors, verticalEncoder, TRACKING_WHEEL),
		{.setpoint = 0, .amplitude = 3000, .maxDeviation = 12, .hysteresis = 0.25}).summary;
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region SelectorScreen //////////////////////////////////
////////////////////////////////////////////////////////////////

// Auton page
const std::vector<tide::screen::Auton> autons = {
	{"X",	xAuton},			// Audience wall, centered on the Toggle, facing it
	{"Left",	leftAuton},		// Alliance wall, centered on the Toggle, facing it
	{"Skills",	skillsAuton},	// Skills only - crosses Auton Line
	{"Test",	testAuton}
};

// Tune page
const std::vector<tide::screen::Tuner> tuners = {
	{"Winch",      tuneWinch},
	{"Angular",    tuneAngular},
	{"Lateral",    tuneLateral},
};

// Info page
std::string robotInfo() {
	using tide::screen::hottest;
	using tide::screen::inches;
	const lemlib::Pose pose = chassis.getPose();
	char text[512];
	std::snprintf(text, sizeof(text),
		"Pose\tx %.1f\ty %.1f\theading %.1f°\n"
		"Distance\tL %s in\tR %s in\tM %s in\n"
		"Winch\t%.0f°\n"
		"Temps\tdrive %.0f °C\twinch %.0f °C\tclaw %.0f °C",
		pose.x, pose.y, pose.theta,
		inches(leftDistance).c_str(), inches(rightDistance).c_str(), inches(middleDistance).c_str(),
		winch.position(),
		std::max(hottest(leftMotors), hottest(rightMotors)), hottest(winchMotors), claw.get_temperature());
	return text;
}

void initialize() {
	tide::screen::init(autons, tuners, robotInfo, {"Nanny"});
	initRobot();
}

#pragma endregion
////////////////////////////////////////////////////////////////
