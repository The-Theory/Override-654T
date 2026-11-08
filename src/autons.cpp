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


constexpr int    SQUARE_MV     = 4000;	// push that squares the bumper on a wall without bouncing off
constexpr int    LOADER_LIFT   = 400;	// winch [deg] lifts a Match Load over the opening's lip before pulling out
constexpr double LOADER_PULL   = 6;		// [in] out of the Loader before turning

constexpr int CLEAR[] = {1000, 1600, 2200, 3300};	// winch [deg] to carry a piece over a goal with 0, 1, 2, 3 Pins on it

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

// Backs straight onto target until the claw sees a piece, then grabs; false if nothing showed up
bool backOnto(tide::Point target) {
	odom.approachStraight(target, STANDOFF);
	const bool seen = odom.creep(-CREEP_MV, clawSeesPiece);	// Claw on the back
	closeClaw();
	odom.creep(-CREEP_MV, [] { return false; }, GRAB_MS);
	if (!seen) std::printf("Grab at (%.1f, %.1f) FAILED: claw saw nothing\n", target.x, target.y);
	return seen;
}

// Picks up a piece off the tiles
// - heading: robot facing at pickup, for pieces boxed in by a goal or wall; else straight from where it is
bool grab(tide::Point piece, std::optional<double> heading = {}) {
	openClaw();
	if (heading) odom.approach(piece, *heading, STANDOFF);
	else         odom.approachStraight(piece, STANDOFF);
	return backOnto(spotted(piece));	// Line up on where it really is
}

// Next Match Load out of a Loader's bottom opening, backing in square to the wall. No scan: the tube
// centers the piece, and its sides would read as objects
bool grabLoad(tide::Point loader) {
	openClaw();
	odom.approach(loader, EAST, STANDOFF);
	odom.wallReset(loader.y < 0 ? Wall::AUDIENCE : Wall::FAR);	// Lines up on the tube
	const bool got = backOnto(loader);
	winch.moveTo(LOADER_LIFT);
	pros::delay(100);
	odom.moveBy(LOADER_PULL);
	return got;
}

// Backs the held piece onto the goal, drops it, pulls away
// - heading: robot facing on the goal; else straight from where it is
void score(tide::Point goal, std::optional<double> heading = {}) {
	if (heading) odom.approach(goal, *heading, 0);
	else         odom.approachStraight(goal, 0);
	winch.moveTo(0);
	pros::delay(350);
	openClaw();
	odom.moveBy(SCORE_BACKOFF);
	stopClaw();
}

// Where the center sits with the front flush on the wall under a Toggle, facing it
tide::Point flushOn(tide::Point toggle) {
	const double heading = tide::bearing({0, 0}, toggle);
	const tide::Point back = tide::toField(0, -FRONT_BUMPER, heading);
	return {toggle.x + back.x, toggle.y + back.y};
}

// Front squared and pressed on the Toggle ahead until it shows our color; also resets the
// coordinate across that wall and the heading
void pressToggle(tide::Point toggle) {
	const double heading = std::fmod(tide::bearing({0, 0}, toggle) + 360, 360);
	const tide::Point flush = flushOn(toggle);
	odom.pushUntilStall(SQUARE_MV, 800, 150);
	const lemlib::Pose at = chassis.getPose();
	const bool wallAlongX = std::fabs(toggle.y) > std::fabs(toggle.x);
	chassis.setPose(wallAlongX ? at.x : flush.x, wallAlongX ? flush.y : at.y, heading);
	odom.moveBy(3, 0, 0, 500, {}, true);	// Keep pressing while it flips
	waitForToggle();
}

// Set down centered on a Toggle, facing it, holding the preload
void startOnToggle(tide::Point toggle) {
	closeClaw();
	const tide::Point flush = flushOn(toggle);
	chassis.setPose(flush.x, flush.y, std::fmod(tide::bearing({0, 0}, toggle) + 360, 360));
	pressToggle(toggle);
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Sections ////////////////////////////////////////
///////////////////////////////////////////////////////////////
// Pieces of routines; not on the selector

// Q1 Toggle, preload and both diagonal cuppins on the Q1 alliance goal; ends holding the outer one
void leftBase() {
	startOnToggle(toggle::Q1);
	winch.moveTo(CLEAR[0]);

	odom.approach(goal::Q1_ALLIANCE, NORTH, STANDOFF);
	odom.wallReset(Wall::RED);
	score(goal::Q1_ALLIANCE, NORTH);

	grab(cuppin::DIAGONAL_INNER);
	winch.moveTo(CLEAR[1]);
	score(goal::Q1_ALLIANCE, EAST);

	grab(cuppin::DIAGONAL_OUTER);
	winch.moveTo(CLEAR[2]);
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Routines ////////////////////////////////////////
////////////////////////////////////////////////////////////////
// Full autons, picked on the selector

void xAuton(tide::Alliance) {
	/// Flip the Q2 Toggle ////////////////////////////////////
	startOnToggle(toggle::Q2);
	winch.moveTo(CLEAR[0]);

	/// Preload on the Q2 alliance goal ///////////////////////
	odom.approach(goal::Q2_ALLIANCE, EAST, STANDOFF);
	odom.wallReset(Wall::AUDIENCE);					// Right beam on the audience wall
	score(goal::Q2_ALLIANCE, EAST);

	/// Audience wall cuppin on top ///////////////////////////
	// Boxed in: Goal 21.5" north, Cups either side, wall behind. Under ~25° the front clips
	// the Goal, past ~30° the back corner hits the wall first
	grab(cuppin::AUDIENCE_WALL_W, NORTH + 27);
	winch.moveTo(CLEAR[1]);
	odom.turnTo(EAST);
	odom.wallReset(Wall::AUDIENCE);					// Before the move, so it lands in line with the goal
	score(goal::Q2_ALLIANCE, EAST);
}

void leftAuton(tide::Alliance) {
	leftBase();
	score(goal::Q1_ALLIANCE, SOUTH);
}

void leftAWP(tide::Alliance) {
	leftBase();
	score(goal::Q1_NEUTRAL);
}

void skillsAuton(tide::Alliance) {
	leftBase();
	score(goal::Q1_ALLIANCE, SOUTH);

	/// Match Loads from the S Loader /////////////////////////
	// The drive team loads a Pin, a Pin, then a Cup + Pin: the Q2 goal has no Cup to hold a Cup + Pin yet
	grabLoad(loader::S);
	winch.moveTo(CLEAR[3]);
	score(goal::Q1_ALLIANCE, SOUTH);

	for (int height : {CLEAR[0], CLEAR[1]}) {
		grabLoad(loader::S);
		winch.moveTo(height);
		score(goal::Q2_ALLIANCE, WEST - 30);			// Backing in from the Loader side, clear of the wall cuppin
	}

	/// Q2 Toggle, then park in Midfield //////////////////////
	chassis.moveToPoint(-30, -35, 1500, {}, false);	// North of the Q2 goal
	chassis.moveToPoint(0, -35, 1500, {}, false);
	odom.turnTo(SOUTH);
	chassis.moveToPoint(0, flushOn(toggle::Q2).y + 3, 1500, {}, false);
	pressToggle(toggle::Q2);
	chassis.moveToPoint(0, 0, 1500, {.forwards = false}, false);
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
