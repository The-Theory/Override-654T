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

#pragma endregion
////////////////////////////////////////////////////////////////

int main() {
	sim::World& w = sim::world();
	w.trackingOffset = OFFSET;
	w.mounts[leftSensor.get_port()]  = LEFT;
	w.mounts[rightSensor.get_port()] = RIGHT;
	w.mounts[frontSensor.get_port()] = FRONT;

	bool done = false;
	pros::Task([&] {
		chassis.calibrate();
		chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
		checkOffset();
		checkMounts();
		done = true;
	});
	w.sched.run([&] { return done; }, [&] { w.step(); });

	std::printf(misses ? "%d missed\n" : "All found\n", misses);
	std::fflush(stdout);
	std::_Exit(misses ? 1 : 0);
}
