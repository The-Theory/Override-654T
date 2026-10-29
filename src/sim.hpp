#pragma once

// 654T in the desktop sim (tools/sim); the brain build never includes this

#include "robot.hpp"
#include "game.hpp"
#include "world.hpp"

////////////////////////////////////////////////////////////////
#pragma region SimRobot ////////////////////////////////////////
////////////////////////////////////////////////////////////////

// What the robot physically is; drivetrain and tracking wheel come from robot.hpp unless set here
inline void simRobot(sim::World& w) {
	w.width = 15;	// [in] TODO: measure
	w.front = 5.2;	// Autons set the center 65" out when flush on a wall
	w.back  = 10;

	// Same as odom's mounts until they're remeasured
	w.mounts[leftDistance.get_port()]   = {0, 0, tide::dir::WEST};
	w.mounts[rightDistance.get_port()]  = {0, 0, tide::dir::EAST};
	w.mounts[middleDistance.get_port()] = {0, 0, tide::dir::NORTH};

	w.joints.push_back({
		.ports          = winchMotors.get_port_all(),
		.sensor         = winchEncoder.get_port(),
		.sensorPerMotor = 0.25,	// Guess
		.gravityMv      = 1000,	// Near the tuned kG
		.min            = 0,
		.max            = 1200,
	});

	// Claw on the back, under the winch
	auto& game = sim::game::robot;
	game.claw.forward = -3;	// Same as odom's toolForward
	game.claw.grab    = [] { return claw.get_voltage() > 0; };
	game.claw.release = [] { return claw.get_voltage() < 0; };
	game.claw.lowered = [] { return winch.position() < 60; };	// Guess

	// activeToggle wheel and colorSensor at the front
	game.spinner.motor   = activeToggle.get_port();
	game.spinner.optical = colorSensor.get_port();
	game.spinner.forward = w.front;

	game.preload = {{.top = sim::game::RED, .bottom = sim::game::YELLOW}};	// Guess: our red/yellow Pin, red up
}

#pragma endregion
////////////////////////////////////////////////////////////////
