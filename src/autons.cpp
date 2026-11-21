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
///////////////////////////////////////////////////////////////
// Pieces of routines; not on the selector

const int TURN_TIMEOUT = 1000;	// [ms] long enough to finish on weak battery

void pickupAt(tide::Point point, int winchHeight, double xOffset=0, double yOffset=0, bool leave=false, bool liftLoader=false, float minSpeed=30, float maxSpeed=127, bool fuckassparameter=false) {
	chassis.turnToPoint(point.x + xOffset, point.y + yOffset, 500, 
		{.forwards=false}, false);
	openClaw();
	chassis.moveToPoint(point.x + xOffset, point.y + yOffset, 1200, 
		{.forwards=false, .maxSpeed=maxSpeed, .minSpeed=minSpeed, .earlyExitRange=15}, false);
	odom.moveBy(-8.5, 0, 0, 1000, {}, true);

	if (fuckassparameter) {
		//odom.moveBy(-8, 1, 0, 1000, {}, false);
		//chassis.turnToHeading(NORTH + 30, 800, {}, false);
		//odom.moveBy(-8, 1, 0, 1000, {}, false);
		
		odom.turnBy(-10);
		odom.moveBy(-36, 0, 0, 500, {.minSpeed=127}, false);
		leftMotors.move_voltage(-10000);
		rightMotors.move_voltage(1000);
	}
		

	while (clawDistance.get_distance() > 85) {
		pros::delay(10);
	}
	leftMotors.move_voltage(0);
	rightMotors.move_voltage(0);

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
		NORTH, 1500, {.forwards = false, .minSpeed=30, .earlyExitRange=5});	
	autonScore(10);	
	odom.wallReset(Wall::RED);								// Wall reset

	/// Scoring innner cup ////////////////////////////////////
	pickupAt(cuppin::DIAGONAL_INNER, 1600, 2, -2);
	odom.reachPose(goal::Q1_ALLIANCE, 
		EAST, 1400, {.forwards=false, .minSpeed=30});
	autonScore(10);

	pickupAt(cuppin::DIAGONAL_OUTER, 2200, 0, -2);
	odom.wallReset(Wall::AUDIENCE);
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Routines ////////////////////////////////////////
////////////////////////////////////////////////////////////////
// Full autons, picked on the selector

void xAuton(tide::Alliance) {
	/// Scoring first cuppin //////////////////////////////////
	// Init
	closeClaw();
	chassis.setPose(0, -65, SOUTH);							// Start pos
	odom.wallReset(Wall::AUDIENCE);								// Wall reset
	
	// Toggle
	odom.moveBy(3, 0, 0, 500, {}, true);
	waitForToggle();										// Toggle to alliance
	winch.moveTo(1000);										// Lift winch							

	// To goal and score
	odom.reachPose(goal::Q2_ALLIANCE,						// Turn to goal
		EAST, 2000, {.forwards = false, .minSpeed=30});	
	autonScore(8);	
	odom.wallReset(Wall::AUDIENCE);								// Wall reset

	/// Pickup aud. cuppin ////////////////////////////////////
	pickupAt(cuppin::AUDIENCE_WALL_W, 1500, -4, 0, false, false, 50, 100, true);
	odom.wallReset(Wall::AUDIENCE);
	chassis.turnToHeading(EAST, 800, {}, false);
	pros::delay(100);
	chassis.setPose(-24, -2.5*24, EAST);
	odom.moveBy(24);
	odom.reachPose(goal::Q2_ALLIANCE,						// Turn to goal
		EAST, 2000, {.forwards = false, .minSpeed=20});	
	autonScore(10);
}

void leftAuton(tide::Alliance) {
	leftBase();

	//// Score aud. cuppin in S Goal ///////////////////////////
	chassis.turnToHeading(SOUTH, 800, {}, false);
	chassis.moveToPoint(goal::Q1_ALLIANCE.x + 2, goal::Q1_ALLIANCE.y, 900, 
		{.forwards=false, .minSpeed=30, .earlyExitRange=5}, false);
	odom.wallReset(Wall::RED);
	autonScore(10);
}

void leftAWP(tide::Alliance) {
	leftBase();

	//// Score aud. cuppin in N Goal ///////////////////////////
	chassis.turnToHeading(WEST-30, 800, {}, false);
	chassis.moveToPoint(goal::Q1_NEUTRAL.x + 2, goal::Q1_NEUTRAL.y, 2500, 
		{.forwards=false, .minSpeed=30, .earlyExitRange=5}, false);
	odom.wallReset(Wall::RED);
	autonScore(10);
}

void skillsAuton(tide::Alliance) {
	/// Run left base /////////////////////////////////////////
	leftBase();

	/// Score aud. cuppin in S Goal ///////////////////////////
	chassis.turnToHeading(SOUTH, 800, {}, false);
	chassis.moveToPoint(goal::Q1_ALLIANCE.x + 2, goal::Q1_ALLIANCE.y, 900, 
		{.forwards=false, .minSpeed=30, .earlyExitRange=5}, false);
	odom.wallReset(Wall::RED);
	autonScore(10);

	/// Scoring first pin /////////////////////////////////////
	chassis.moveToPose(-46, -62, EAST, 2000, {}, false);
	pickupAt(loader::RED_S, 3300, 3, 3, true, true);
	pros::delay(100);
	chassis.moveToPose(-46, -62, SOUTH, 2000, {}, false);
	chassis.moveToPoint(goal::Q1_ALLIANCE.x, goal::Q1_ALLIANCE.y, 1000, 
		{.forwards=false, .minSpeed=30, .earlyExitRange=5}, false);
	odom.wallReset(Wall::RED);
	autonScore(10);

	// /// Scoring second pin ////////////////////////////////////
	// chassis.moveToPose(-46, -62, EAST, 2000, {}, false);
	// pickupAt(loader::RED_S, 4500, 3, 3, true, true);
	// pros::delay(100);
	// chassis.moveToPose(-46, -62, SOUTH, 2000, {}, false);
	// chassis.moveToPoint(goal::Q1_ALLIANCE.x+1, goal::Q1_ALLIANCE.y, 1000,
	// 	{.forwards=false, .minSpeed=30, .earlyExitRange=5}, false);
	// odom.wallReset(Wall::RED);
	// autonScore(10);

	/// Scoring Three cuppins on second goal //////////////////
	for (int winch_height : {1000, 1600}) { // 2900, 3700
		chassis.moveToPose(-46, -62, EAST, 2000, {}, false);    				// Swirve to face loader 
		pickupAt(loader::RED_S, winch_height, 3, 3, true,true);					// Pickup loader cuppin
		pros::delay(100);														
		chassis.moveToPose(-46, -62, SOUTH, 2000, {}, false);
		chassis.moveToPoint(goal::Q2_ALLIANCE.x + 3, goal::Q2_ALLIANCE.y - 1, 1500,
			{.forwards=false, .minSpeed=30, .earlyExitRange=5}, false);
		odom.wallReset(Wall::AUDIENCE);
		autonScore(6);
	}	

	/// Go around to toggle  //////////////////////////////////
	chassis.moveToPose(-48, -60, WEST, 1500, {}, false); 
	chassis.moveToPose(0, -24, SOUTH, 1500, {.forwards=false}, false);
	chassis.moveToPose(0, -60, SOUTH, 1500, {}, false);
	waitForToggle();
	chassis.moveToPoint(0, 0, 1000);
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
		{.setpoint = 0, .amplitude = 8000, .maxDeviation = 45}).summary;
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
	{"Left AWP", leftAWP}, 		// Alliance wall - Scores on neutral for AWP
	{"Skills",	skillsAuton},	// Skills only - crosses Auton Line
	{"Left",	leftAuton},		// Alliance wall, centered on the Toggle, facing it
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
		inches(leftDistance).c_str(), inches(rightDistance).c_str(), inches(frontDistance).c_str(),
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
