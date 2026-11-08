#include "macros.hpp"
#include "tide/autotune.hpp"
#include "tide/override.hpp"
#include "tide/screen.hpp"
#include <algorithm>
#include <optional>
#include <cstdio>

using namespace tide::dir;
using namespace tide::override;
using tide::Wall;

////////////////////////////////////////////////////////////////
#pragma region Steps ///////////////////////////////////////////
////////////////////////////////////////////////////////////////
// One thing each, few arguments. A field that differs is handled by what the robot
// senses, not by offsets; routines compose these instead of growing flags

constexpr double STANDOFF      = 8;		// [in] tool stops this short of a piece or goal, then finishes slowly
constexpr int    CREEP_MV      = 3000;	// slow enough to stop on the claw sensor
constexpr int    GRAB_MM       = 85;	// claw sensor reading once a piece is in the mouth
constexpr int    GRAB_MS       = 300;	// keeps backing while the claw closes, so the piece seats
constexpr double SCORE_BACKOFF = 8;		// [in] clear of the stack before turning
constexpr int    SCORE_SPEED   = 35;	// [/127] onto the goal; slow enough to stop on it instead of rolling past

constexpr double START_Y        = -70.2 + FRONT_BUMPER;	// [in] front flush on the audience wall
constexpr int SQUARE_MV        = 4000;	// push that squares the bumper on a wall without bouncing off

constexpr int CLEAR_EMPTY_GOAL = 1000;	// winch [deg] to carry a piece over a goal
constexpr int CLEAR_ONE_PIN    = 1500;

bool clawSeesPiece() { return clawDistance.get_distance() < GRAB_MM; }

// Where the claw sensor sees the piece meant to be at nominal: the scanned object nearest it, so a
// shifted piece or robot still lines up, and neighbors (wall Cups) aren't mistaken for it
tide::Point spotted(tide::Point nominal) {
	constexpr double SWEEP = 20;		// [deg] either side
	constexpr double PIECE_R = 1.55;	// [in] Pin flange / Cup rim
	constexpr double TRUST = 3;			// [in] further off than this is something else
	const std::vector<tide::Point> seen = odom.scan(clawMount, SWEEP, PIECE_R);
	const tide::Point* best = nullptr;
	for (const tide::Point& p : seen)
		if (!best || std::hypot(p.x - nominal.x, p.y - nominal.y) < std::hypot(best->x - nominal.x, best->y - nominal.y)) best = &p;
	if (!best || std::hypot(best->x - nominal.x, best->y - nominal.y) > TRUST) {
		std::printf("Scan at (%.1f, %.1f): nothing near, using the nominal spot\n", nominal.x, nominal.y);
		return nominal;
	}
	std::printf("Scan at (%.1f, %.1f): piece %+.1f, %+.1f in off\n", nominal.x, nominal.y, best->x - nominal.x, best->y - nominal.y);
	return *best;
}

// Backs onto a piece until the claw sees it, then grabs; false if nothing showed up
// - heading: robot facing at pickup, for pieces boxed in by a goal or wall; else straight from where it is
bool grab(tide::Point piece, std::optional<double> heading = {}) {
	openClaw();
	if (heading) odom.approach(piece, *heading, STANDOFF);
	else         odom.approachStraight(piece, STANDOFF);
	odom.approachStraight(spotted(piece), STANDOFF);	// Line up on where it really is
	const bool seen = odom.creep(-CREEP_MV, clawSeesPiece);	// Claw on the back
	closeClaw();
	odom.creep(-CREEP_MV, [] { return false; }, GRAB_MS);
	if (!seen) std::printf("Grab at (%.1f, %.1f) FAILED: claw saw nothing\n", piece.x, piece.y);
	return seen;
}

// From a standoff in front of the goal: backs the held piece onto it, drops it, pulls away
void score(tide::Point goal, double heading) {
	odom.approach(goal, heading, 0, SCORE_SPEED);
	winch.moveTo(0);
	pros::delay(350);
	openClaw();
	odom.moveBy(SCORE_BACKOFF);
	stopClaw();
}

#pragma endregion
////////////////////////////////////////////////////////////////

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
	/// Flip the Q2 Toggle ////////////////////////////////////
	closeClaw();									// Hold the preload
	chassis.setPose(0, START_Y, SOUTH);				// Centered on the Toggle, facing it; the front beam reads the Toggle, so no reset here
	odom.pushUntilStall(SQUARE_MV, 800, 150);		// Flush on it: squares the heading, however it was set down
	chassis.setPose(chassis.getPose().x, START_Y, SOUTH);
	odom.moveBy(3, 0, 0, 500, {}, true);			// Keep pressing while it flips
	waitForToggle();
	winch.moveTo(CLEAR_EMPTY_GOAL);

	/// Preload on the Q2 alliance goal ///////////////////////
	odom.approach(goal::Q2_ALLIANCE, EAST, STANDOFF);
	odom.wallReset(Wall::AUDIENCE);					// Right beam on the audience wall
	score(goal::Q2_ALLIANCE, EAST);

	/// Audience wall cuppin on top ///////////////////////////
	// Boxed in: Goal 21.5" north, Cups either side, wall behind. Under ~25° the front clips
	// the Goal, past ~30° the back corner hits the wall first
	grab(cuppin::AUDIENCE_WALL_W, NORTH + 27);
	winch.moveTo(CLEAR_ONE_PIN);
	odom.turnTo(EAST);
	odom.wallReset(Wall::AUDIENCE);					// Before the move, so it lands in line with the goal
	odom.approach(goal::Q2_ALLIANCE, EAST, STANDOFF);
	score(goal::Q2_ALLIANCE, EAST);
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

// Drive and turn feedforward, lift down then up, since a raised lift is more inertia.
// ~2 ft clear front and back; paste each into the matching tide::Feedforward
std::string characterizeAt(int lift) {
	winch.moveTo(lift);
	pros::delay(1500);
	const char* pose = lift == 0 ? "down" : "up";
	char lateral[16], angular[16];
	std::snprintf(lateral, sizeof(lateral), "lateral-%s", pose);
	std::snprintf(angular, sizeof(angular), "angular-%s", pose);
	std::string out = tide::characterize(lateral, tide::lateralPlant(leftMotors, rightMotors, verticalEncoder, TRACKING_WHEEL));
	out += tide::characterize(angular, tide::angularPlant(leftMotors, rightMotors, imu));
	winch.moveTo(0);
	return out;
}
std::string ffLiftDown() { return characterizeAt(0); }
std::string ffLiftUp()   { return characterizeAt(LIFT_LOADED); }

// Facing a wall a few inches off it, room to spin; after Mounts. Paste into FRONT_BUMPER / BACK_BUMPER
std::string tuneBumpers() { return odom.measureBumpers(0); }

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
	{"Bumpers",    tuneBumpers},
	{"FF down",    ffLiftDown},
	{"FF up",      ffLiftUp},
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
