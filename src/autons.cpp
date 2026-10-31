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
#pragma region Sections ////////////////////////////////////////
////////////////////////////////////////////////////////////////
// Pieces of routines; not on the selector

const int TURN_TIMEOUT = 1000;	// [ms] long enough to finish on weak battery

void resetEndSegment() {
	odom.moveBy(tide::TILE/2);
	pros::delay(800);
	winch.moveTo(0);
	pros::delay(800);
}

void scoreSegment() {
	winch.moveTo(750);			// Lower winch to score
	openClaw();					// Release pin
	pros::delay(1000);			// Wait for pin to drop
	odom.moveBy(12);			// Drive away from goal
	winch.moveTo(0);			// Put winch down
	stopClaw();					// Close claw
}

// Flush on the audience wall facing the Q2 Toggle: flips it, then scores the preload in Q2 alliance
void q2Opening() {
	// Init
	spinClaw();
	chassis.setPose(0, -65, SOUTH);	// Start pos
	
	// Toggle
	waitForToggle();				// Toggle to alliance
	odom.wallReset(Wall::AUDIENCE);	// Wall reset
	pros::delay(500);
	winch.moveTo(1000);				// Lift winch							

	// To goal and score
	odom.reachPose(goal::Q2_ALLIANCE, 		// Turn to goal
		EAST, 2000, {.forwards = false});	
	odom.moveBy(1, 0, 0, 300);
	winch.moveBy(-500);
	pros::delay(300);						// Wait to settle at goal
	odom.wallReset(Wall::AUDIENCE);			// Reset
	openClaw();								// Release pin
	pros::delay(800);						// Wait for pun to drop
	stopClaw();								// Close claw again
}

void leftBase() {
	chassis.setPose(-65, 0, WEST);	// Flush on the red wall, facing it
	pros::delay(100);
	waitForToggle();
	odom.wallReset(Wall::RED);

	// To goal
	pros::delay(100);
	winch.moveTo(60);
	odom.reachPose(goal::Q1_ALLIANCE, NORTH, 2000, {.forwards = false, .minSpeed = 50});
	pros::delay(150);
	openClaw();
	pros::delay(230);

	// To cuppin
	winch.moveTo(400);
	chassis.moveToPose(-48, -10, NORTH, 1000, {.minSpeed=50});
	odom.reachPose(cuppin::DIAGONAL_INNER, NORTH_WEST, 1500, {.forwards = false, .minSpeed = 40});

	// Pickup cuppin
	spinClaw();
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
	openClaw();
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
	spinClaw();
	winch.moveTo(30);
	pros::delay(1000);
	winch.moveTo(700);
	stopClaw();

	// Score
	odom.aim(goal::Q1_ALLIANCE, TURN_TIMEOUT);
	odom.reachPoint(goal::Q1_ALLIANCE, 800, {.maxSpeed = 100});
	winch.moveTo(15);
	pros::delay(400);
	openClaw();
	pros::delay(800);
	stopClaw();
	winch.moveTo(1000);
	pros::delay(500);

	// Back out
	chassis.moveToPoint(-48, -48, 1500, {}, false);
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Routines ////////////////////////////////////////
////////////////////////////////////////////////////////////////
// Full autons, picked on the selector

void xAuton(tide::Alliance) {
	q2Opening();

    // Back off goal, ready to pick up
    odom.moveBy(6);                                             // Back up from Q2 alliance
    odom.wallReset(Wall::AUDIENCE);                                 // Reset position
    winch.moveTo(0);                                                // Put winch down
    spinClaw();                                                     // Spin claw to pick up cuppin

    // Get West Audience Wall Cuppin
    odom.aim(cuppin::AUDIENCE_WALL_W);                              // Aim to cuppin
    odom.turnBy(8);                                             // Add turn offset
    odom.moveBy(-tide::TILE * 0.9, 0, 0, 1500, {.maxSpeed=100});    // Move to cuppin
    odom.moveBy(-tide::TILE * 0.2, 0, 0, 1500, {.maxSpeed=20});     // Move to cuppin
    pros::delay(1000);                                              // Wait for intake to get sucked up
    winch.moveTo(1400);                                             // Lift cuppin up

    // Turn out
    chassis.turnToHeading(EAST, 800, {}, false);                    // Turn out
    odom.moveBy(tide::TILE/2);                                      // Drive out

    // Score    
    odom.reachPoint(goal::Q2_ALLIANCE);                             // Move to goal
    scoreSegment();                                                 // Score cuppin

    // Get new cuppin
    odom.aim(cuppin::MIDFIELD_S);                                   // Aim away
    odom.moveBy(-24);                                               // Get to a middle point
    odom.aim(cuppin::AUDIENCE_WALL_E);                              // Turn to new cuppin
    spinClaw();                                                     // Spin claw
    odom.moveBy(tide::TILE * -2, 0, 0, 2000, {.maxSpeed=40});   // Move to cuppin

    pros::delay(1000);                                              // Wait for cuppin to get in claw
    winch.moveTo(1300);                                             // Lift cuppin up
    chassis.turnToHeading(WEST, 800, {}, false);                    // Turn
    odom.moveBy(tide::TILE/2);                                      // Drive out

    // Score    
    odom.reachPoint(goal::Q2_NEUTRAL);                              // Get to goal
    scoreSegment();                                                 // Score cuppin

    resetEndSegment();
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
	spinClaw();
	chassis.moveToPose(-24, 32, NORTH, 2000, {}, false);
	pros::delay(4000);

	// Score
	winch.moveTo(250);
	stopClaw();
	odom.reachPoint(goal::MIDFIELD);
	openClaw();
}

void testAuton(tide::Alliance) {
	q2Opening();
	// odom.moveBy(24);
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Tuning //////////////////////////////////////////
////////////////////////////////////////////////////////////////

std::string tuneWinch() {
	const tide::TuneResult result = tide::autotune("winch", tide::mechanismPlant(winch), 
		{.setpoint = 400, .amplitude = 4'000, .maxDeviation = 200, .hysteresis = 1});
	winch.moveTo(0);
	return result.summary;
}

// Needs ~2 ft clear around the robot
std::string tuneAngular() {
	return tide::autotune("angular", tide::angularPlant(leftMotors, rightMotors, imu), 
		{.setpoint = 0, .amplitude = 4000, .maxDeviation = 45}).summary;
}

// Needs ~1 ft clear in front and behind. A big swing so it measures driving, not friction
std::string tuneLateral() {
	return tide::autotune("lateral", tide::lateralPlant(leftMotors, rightMotors, verticalEncoder, TRACKING_WHEEL),
		{.setpoint = 0, .amplitude = 7000, .maxDeviation = 12, .hysteresis = 0.25, .rule = tide::TuneRule::SOME_OVERSHOOT}).summary;
}

// Spins 5 turns in place; paste the result into verticalWheel
std::string tuneOffset() {
	return tide::trackingOffset(leftMotors, rightMotors, imu, verticalEncoder, TRACKING_WHEEL);
}

// Front a few inches from a wall, room to spin; paste the results into odom's mounts
std::string tuneMounts() {
	constexpr double SENSOR_WIDTH = 11.22047;	// [in]
	return odom.measureMounts(0, 1, SENSOR_WIDTH, 6000);
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region SelectorScreen //////////////////////////////////
////////////////////////////////////////////////////////////////

// Auton page
const std::vector<tide::screen::Auton> autons = {
	{"X",		xAuton},		// Audience wall, centered on the Toggle, facing it
	{"Test",	testAuton},		// For testing only
	{"Left",	leftAuton},		// Alliance wall, centered on the Toggle, facing it
	{"Skills",	skillsAuton},	// Skills only - crosses Auton Line
};

// Tune page
const std::vector<tide::screen::Tuner> tuners = {
	{"Winch",      tuneWinch},
	{"Angular",    tuneAngular},
	{"Lateral",    tuneLateral},
	{"Offset",     tuneOffset},
	{"Mounts",     tuneMounts},
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
		"Toggle\thue %.0f°\tnear %d\t%s\n"
		"Temps\tdrive %.0f °C\twinch %.0f °C\tclaw %.0f °C",
		pose.x, pose.y, pose.theta,
		inches(leftDistance).c_str(), inches(rightDistance).c_str(), inches(middleDistance).c_str(),
		winch.position(),
		colorSensor.get_hue(), colorSensor.get_proximity(), toggleShowsAlliance() ? "ours" : "not ours",
		std::max(hottest(leftMotors), hottest(rightMotors)), hottest(winchMotors), claw.get_temperature());
	return text;
}

void initialize() {
	tide::screen::init(autons, tuners, robotInfo, {"Nanny"});
	initRobot();
	openClaw();
	pros::Task{autoToggle}; // Start autoToggling
}

#pragma endregion
////////////////////////////////////////////////////////////////
