// Tide's tuners on a made-up robot whose real offsets are known: each must find them.
// Exits nonzero on a miss.

#include "main.h"
#include "lemlib/api.hpp"
#include "tide/autotune.hpp"
#include "tide/odom.hpp"
#include "world.hpp"
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace tide::dir;
using tide::Feedforward;

////////////////////////////////////////////////////////////////
#pragma region Robot ///////////////////////////////////////////
////////////////////////////////////////////////////////////////

pros::MotorGroup leftMotors({1, 2}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({3, 4}, pros::MotorGearset::blue);
pros::Imu imu(5);
pros::Rotation tracking(6);
pros::Distance leftSensor(7), rightSensor(8), frontSensor(9);

constexpr float WHEEL = lemlib::Omniwheel::NEW_2;
lemlib::TrackingWheel wheel(&tracking, WHEEL, 0);	// Offset unknown to the code
lemlib::Chassis chassis(
	lemlib::Drivetrain(&leftMotors, &rightMotors, 12, lemlib::Omniwheel::NEW_275, 450, 2),
	lemlib::ControllerSettings(8, 0, 30, 0, 1, 100, 3, 500, 0),
	lemlib::ControllerSettings(2, 0, 12, 0, 1, 100, 3, 500, 0),
	lemlib::OdomSensors(&wheel, nullptr, nullptr, nullptr, &imu));

tide::Odom odom(chassis, leftMotors, rightMotors, tracking, {
	{leftSensor,  0, 0, WEST},
	{rightSensor, 0, 0, EAST},
	{frontSensor, 0, 0, NORTH},
});

// Where things really are [in]
constexpr double OFFSET = 1.25;
const sim::Mount LEFT  = {-5.5,  1.0, WEST};
const sim::Mount RIGHT = { 5.75, 1.0, EAST};
const sim::Mount FRONT = { 0.5,  6.0, NORTH};

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Checks //////////////////////////////////////////
////////////////////////////////////////////////////////////////

int misses = 0;

void expect(const char* what, double got, double want, double tolerance) {
	const bool ok = std::fabs(got - want) <= tolerance;
	if (!ok) misses++;
	std::printf("%-4s %-16s got %6.2f  want %6.2f\n", ok ? "ok" : "MISS", what, got, want);
}

void checkOffset() {
	chassis.setPose(0, 0, 0);
	const std::string out = tide::trackingOffset(leftMotors, rightMotors, imu, tracking, WHEEL);
	double offset = NAN;
	std::sscanf(out.c_str(), "Tracking offset %lf", &offset);
	expect("tracking offset", offset, OFFSET, 0.05);
}

// Starts backed up near the audience wall, like the tuner asks
void checkMounts() {
	sim::World& w = sim::world();
	w.placed = false;
	chassis.setPose(0, -70.2 + w.back + 3, NORTH);
	const std::string out = odom.measureMounts(0, 1, RIGHT.right - LEFT.right);

	const sim::Mount* want[] = {&LEFT, &RIGHT, &FRONT};
	std::size_t at = 0;
	for (int k = 0; k < 3; k++) {
		int n;
		double right = NAN, forward = NAN;
		at = out.find('\n', at) + 1;
		std::sscanf(out.c_str() + at, "%d: right %lf  forward %lf", &n, &right, &forward);
		char name[24];
		std::snprintf(name, sizeof(name), "mount %d right", k + 1);
		expect(name, right, want[k]->right, 0.1);
		std::snprintf(name, sizeof(name), "mount %d forward", k + 1);
		expect(name, forward, want[k]->forward, 0.1);
	}
}

// Drive model: each side's mv = stiction + (12000 - stiction) / topSpeed · (v + tau · a)
Feedforward truth(double perSideSpeed);

void checkFeedforward(const char* what, const tide::Plant& plant, double perSideSpeed) {
	sim::World& w = sim::world();
	const double kV = truth(perSideSpeed).kV;
	Feedforward ff;
	std::sscanf(tide::characterize(what, plant).c_str(), "FF %*s kS %lf  kV %lf  kA %lf", &ff.kS, &ff.kV, &ff.kA);
	char name[32];
	std::snprintf(name, sizeof(name), "%s kS", what);
	expect(name, ff.kS, w.driveStiction, 0.03 * w.driveStiction);
	std::snprintf(name, sizeof(name), "%s kV", what);
	expect(name, ff.kV, kV, 0.03 * kV);
	std::snprintf(name, sizeof(name), "%s kA", what);
	expect(name, ff.kA, kV * w.driveTau, 0.05 * kV * w.driveTau);
}

void checkFeedforwards() {
	sim::World& w = sim::world();
	w.placed = false;	// Mid-field, clear of walls
	chassis.setPose(0, 0, 0);
	const double degToSide = w.trackWidth / 2 * std::numbers::pi / 180;	// side speed per deg/s spinning
	checkFeedforward("lateral", tide::lateralPlant(leftMotors, rightMotors, tracking, WHEEL), 1);
	checkFeedforward("angular", tide::angularPlant(leftMotors, rightMotors, imu), degToSide);
	w.driveTau = 0.4;	// Heavier robot, as with a lift up: only kA should move
	checkFeedforward("heavy", tide::lateralPlant(leftMotors, rightMotors, tracking, WHEEL), 1);
	w.driveTau = 0.25;
}

// The check robot's true feedforward, straight [in] or turning [deg]
Feedforward truth(double perSideSpeed) {
	sim::World& w = sim::world();
	const double kV = (12000 - w.driveStiction) / (w.wheelRpm * std::numbers::pi * w.wheelDiameter / 60) * perSideSpeed;
	return {w.driveStiction, kV, kV * w.driveTau};
}

// Planned moves land where asked, also on a heavier robot (lift up) with its loaded gains
void checkProfiled() {
	sim::World& w = sim::world();
	w.trackingOffset = 0;	// The code's wheel offset is 0 here; a real one is checkOffset's job
	w.placed = false;
	chassis.setPose(0, 0, 0);
	const double degToSide = w.trackWidth / 2 * std::numbers::pi / 180;
	tide::OdomSettings settings;
	settings.lateral = truth(1);
	settings.angular = truth(degToSide);
	tide::Odom planned(chassis, leftMotors, rightMotors, tracking, {}, settings);

	auto run = [&](const char* what, auto move, double wantY, double wantTheta) {
		const std::uint32_t start = pros::millis();
		move();
		const lemlib::Pose p = chassis.getPose();
		std::printf("     %-16s %.2f s\n", what, (pros::millis() - start) / 1000.0);
		char name[32];
		std::snprintf(name, sizeof(name), "%s y", what);
		expect(name, p.y, wantY, 0.3);
		std::snprintf(name, sizeof(name), "%s heading", what);
		expect(name, std::remainder(p.theta - wantTheta, 360.0) + wantTheta, wantTheta, 1);
	};
	run("drive 24", [&] { planned.profiledDrive(24); }, 24, 0);
	run("drive -36", [&] { planned.profiledDrive(-36); }, -12, 0);
	run("turn 90", [&] { planned.profiledTurn(90); }, -12, 90);
	run("turn back", [&] { planned.profiledTurn(0); }, -12, 0);

	// Lift up: a heavier robot, told so through load()
	w.driveTau = 0.4;
	settings.lateralLoaded = truth(1);
	settings.angularLoaded = truth(degToSide);
	w.driveTau = 0.25;
	settings.load = [] { return 1.0; };
	tide::Odom loaded(chassis, leftMotors, rightMotors, tracking, {}, settings);
	w.driveTau = 0.4;
	run("heavy drive 24", [&] { loaded.profiledDrive(24); }, 12, 0);
	run("heavy turn 90", [&] { loaded.profiledTurn(90); }, 12, 90);
	w.driveTau = 0.25;
}

// Starts facing the far wall a few inches off it, like the tuner asks
void checkBumpers() {
	sim::World& w = sim::world();
	const double front = w.front, back = w.back;
	w.trackingOffset = 0;
	w.placed = false;
	chassis.setPose(0, 70.2 - front - 3, NORTH);
	tide::Odom measured(chassis, leftMotors, rightMotors, tracking, {
		{leftSensor, LEFT.right, LEFT.forward, WEST}, {rightSensor, RIGHT.right, RIGHT.forward, EAST}});
	double gotFront = NAN, gotBack = NAN;
	std::sscanf(measured.measureBumpers(0).c_str(), "Bumpers from center: front %lf  back %lf", &gotFront, &gotBack);
	expect("front bumper", gotFront, front, 0.15);
	expect("back bumper", gotBack, back, 0.15);
}

#pragma endregion
////////////////////////////////////////////////////////////////

int main() {
	sim::World& w = sim::world();
	w.trackingOffset = OFFSET;
	w.mounts[leftSensor.get_port()]  = LEFT;
	w.mounts[rightSensor.get_port()] = RIGHT;
	w.mounts[frontSensor.get_port()] = FRONT;
	w.front = 5.5;	// Bumpers the code doesn't know
	w.back  = 11;
	w.width = 14;

	bool done = false;
	pros::Task([&] {
		chassis.calibrate();
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
		checkOffset();
		checkMounts();
		checkFeedforwards();
		checkProfiled();
		checkBumpers();
		done = true;
	});
	w.sched.run([&] { return done; }, [&] { w.step(); });

	std::printf(misses ? "%d missed\n" : "All found\n", misses);
	std::fflush(stdout);
	std::_Exit(misses ? 1 : 0);
}
