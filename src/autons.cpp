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

void pickupAt(tide::Point point, int winchHeight, double xOffset=0, double yOffset=0, bool leave=false, bool liftLoader=false) {
	chassis.turnToPoint(point.x + xOffset, point.y + yOffset, 500, 
		{.forwards=false}, false);
	openClaw();
	chassis.moveToPoint(point.x + xOffset, point.y + yOffset, 1200, 
		{.forwards=false, .minSpeed=30, .earlyExitRange=15}, false);
	odom.moveBy(-8.5, 0, 0, 1000, {}, true);

	while (clawDistance.get_distance() > 85) {
		pros::delay(10);
	}

	closeClaw();
	if (leave) {
		if (liftLoader) {
			winch.moveTo(400);
			pros::delay(100);
		}
		odom.moveBy(6);
		pros::delay(100);
	}
	winch.moveTo(winchHeight);
	pros::delay(100);
}

void resetEndSegment() {
	odom.moveBy(tide::TILE/2);
	pros::delay(800);
	winch.moveTo(0);
	pros::delay(800);
}

void autonScore(int backup=6) {
	winch.moveTo(0);			// Move down
	pros::delay(350);		
	openClaw();					// Release pin
	pros::delay(350);			// Wait for pin to release
	odom.moveBy(backup);   		// Move out
	stopClaw();					// Stop claw
}

void leftBase() {
	/// Scoring first pin /////////////////////////////////////
	// Init
	closeClaw();
	chassis.setPose(-65, 0, WEST);							// Start pos
	odom.wallReset(Wall::RED);								// Wall reset
	
	// Toggle
	waitForToggle();										// Toggle to alliance
	winch.moveTo(1000);										// Lift winch							

	// To goal and score
	odom.reachPose(goal::Q1_ALLIANCE,						// Turn to goal
		NORTH, 2000, {.forwards = false, .minSpeed=30});	
	autonScore(10);	
	odom.wallReset(Wall::RED);								// Wall reset

	/// Scoring innner cup ////////////////////////////////////
	pickupAt(cuppin::DIAGONAL_INNER, 1600, 2, -2);
	odom.reachPose(goal::Q1_ALLIANCE, 
		EAST, 1400, {.forwards=false, .minSpeed=30});
	autonScore(10);

	/// Scoring aud. cup //////////////////////////////////////
	pickupAt(cuppin::DIAGONAL_OUTER, 2200, 0, -2);
	odom.wallReset(Wall::AUDIENCE);

	chassis.turnToHeading(SOUTH, 800, {}, false);
	chassis.moveToPoint(goal::Q1_ALLIANCE.x + 2, goal::Q1_ALLIANCE.y, 900, 
		{.forwards=false, .minSpeed=30, .earlyExitRange=5}, false);
	odom.wallReset(Wall::RED);
	autonScore(10);
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Routines ////////////////////////////////////////
////////////////////////////////////////////////////////////////
// Full autons, picked on the selector

void xAuton(tide::Alliance) {
	/// Scoring first pin /////////////////////////////////////
	// Init
	closeClaw();
	chassis.setPose(0, -65, SOUTH);	// Start pos
	odom.wallReset(Wall::AUDIENCE);	// Wall reset
	
	// Toggle
	waitForToggle();				// Toggle to alliance
	winch.moveTo(1000);				// Lift winch							

	// To goal and score
	odom.reachPose(goal::Q2_ALLIANCE, 		// Turn to goal
		EAST, 2000, {.forwards = false, .minSpeed=30});	
	autonScore(10);	
	odom.wallReset(Wall::AUDIENCE);	// Wall reset

    // Get West Audience Wall Cuppin
    odom.aim(cuppin::AUDIENCE_WALL_W, 600);    // Aim to cuppin
    odom.moveBy(-tide::TILE * 0.9, 0, 0, 1500, {.maxSpeed=100});    // Move to cuppin
    odom.moveBy(-tide::TILE * 0.2, 0, 0, 300, {.maxSpeed=20});     	// Move to cuppin
    winch.moveTo(1600);                                             // Lift cuppin up

    // // Turn out
    // chassis.turnToHeading(EAST, 600, {}, false);                    // Turn out
	// chassis.setPose(-tide::TILE, -2.5 * tide::TILE, EAST);
    // //odom.moveBy(tide::TILE);                                    	// Drive out
	// chassis.moveToPose(5, -48, EAST, 1000, {}, false);

    // // Score    
	// winch.moveTo(1400);  
	// odom.aim(goal::Q2_ALLIANCE, 800);
    // odom.reachPoint(goal::Q2_ALLIANCE, 800);                       // Move to goal
	// odom.moveBy(1, 0, 0, 300);

    // // Get new cuppin
	// closeClaw();   
	// //odom.reachPoint(tide::Point(0, -50), 1500);                   
    // //odom.reachPoint(cuppin::AUDIENCE_WALL_E, 1500);         

}

void leftAuton(tide::Alliance) {
	leftBase();

	/// Scoring Wall cup //////////////////////////////////////
	pickupAt(cuppin::RED_WALL_S, 3000, 3.5, -4);
}

void skillsAuton(tide::Alliance) {
	/// Run left base /////////////////////////////////////////
	leftBase();

	/// Scoring first pin /////////////////////////////////////
	chassis.moveToPose(-46, -62, EAST, 2000, {}, false);
	pickupAt(loader::RED_S, 2900, 1, 2.5, true, true);
	pros::delay(100);
	chassis.moveToPose(-46, -62, SOUTH, 2000, {}, false);
	chassis.moveToPoint(goal::Q1_ALLIANCE.x, goal::Q1_ALLIANCE.y, 1000, 
		{.forwards=false, .minSpeed=30, .earlyExitRange=5}, false);
	odom.wallReset(Wall::RED);
	autonScore(10);

	/// Scoring second pin ////////////////////////////////////
	chassis.moveToPose(-46, -62, EAST, 2000, {}, false);
	pickupAt(loader::RED_S, 3700, 1, 3, true, true);
	pros::delay(100);
	chassis.moveToPose(-46, -62, SOUTH, 2000, {}, false);
	chassis.moveToPoint(goal::Q1_ALLIANCE.x+1, goal::Q1_ALLIANCE.y, 1000,
		{.forwards=false, .minSpeed=30, .earlyExitRange=5}, false);
	odom.wallReset(Wall::RED);
	autonScore(10);

	/// Scoring first solo-pin on second goal /////////////////
	chassis.moveToPose(-46, -62, EAST, 2000, {}, false);
	pickupAt(loader::RED_S, 1000, 2, 3, true);
	pros::delay(100);
	chassis.moveToPose(-46, -62, WEST, 2000, {}, false);
	chassis.moveToPoint(goal::Q2_ALLIANCE.x, goal::Q2_ALLIANCE.y, 1000,
		{.forwards=false, .minSpeed=30, .earlyExitRange=5}, false);
	odom.wallReset(Wall::AUDIENCE);
	autonScore(6);

	/// Scoring first cuppin on second goal ///////////////////
	chassis.moveToPose(-46, -62, EAST, 2000, {}, false);
	pickupAt(loader::RED_S, 1600, 1, 3, true,true);
	pros::delay(100);
	chassis.moveToPose(-46, -62, WEST, 2000, {}, false);
	chassis.moveToPoint(goal::Q2_ALLIANCE.x, goal::Q2_ALLIANCE.y, 1000,
		{.forwards=false, .minSpeed=30, .earlyExitRange=5}, false);
	odom.wallReset(Wall::AUDIENCE);
	autonScore(6);

	/// Scoring second cuppin on second goal ///////////////////
	chassis.moveToPose(-46, -62, EAST, 2000, {}, false);
	pickupAt(loader::RED_S, 2900, 1, 3, true,true);
	pros::delay(100);
	chassis.moveToPose(-46, -62, WEST, 2000, {}, false);
	chassis.moveToPoint(goal::Q2_ALLIANCE.x, goal::Q2_ALLIANCE.y, 1000,
		{.forwards=false, .minSpeed=30, .earlyExitRange=5}, false);
	odom.wallReset(Wall::AUDIENCE);
	autonScore(6);

	/// Scoring third cuppin on second goal ///////////////////
	chassis.moveToPose(-46, -62, EAST, 2000, {}, false);
	pickupAt(loader::RED_S, 3700, 1, 3, true,true);
	pros::delay(100);
	chassis.moveToPose(-46, -62, WEST, 2000, {}, false);
	chassis.moveToPoint(goal::Q2_ALLIANCE.x, goal::Q2_ALLIANCE.y, 1000,
		{.forwards=false, .minSpeed=30, .earlyExitRange=5}, false);
	odom.wallReset(Wall::AUDIENCE);
	autonScore(6);

	
}

void testAuton(tide::Alliance) {
	// odom.moveBy(24);
}

void blankAuto(tide::Alliance) {}

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
	{"Skills",	skillsAuton},	// Skills only - crosses Auton Line
	{"Left",	leftAuton},		// Alliance wall, centered on the Toggle, facing it
	{"X",		xAuton},		// Audience wall, centered on the Toggle, facing it
	{"Test",	testAuton},		// For testing only
	{"Blank", 	blankAuto},		// Does nothing except toggle
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
	pros::Task{autoToggle}; // Start autoToggling
}

#pragma endregion
////////////////////////////////////////////////////////////////
