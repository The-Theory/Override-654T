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
	w.width = 13.6;			// [in]
	w.front = 5.2;			// Autons set the center 65" out when flush on a wall
	w.back  = 15.718 - w.front;	// 15.718" long overall

	for (const tide::DistanceMount& m : odom.getMounts()) w.mounts[m.sensor.get_port()] = {m.right, m.forward, m.facing};

	w.liftTau = 0.1;	// Guess: a raised cascade slows the drive's response this much

	w.joints.push_back({
		.ports          = winchMotors.get_port_all(),
		.sensor         = winchEncoder.get_port(),
		.sensorPerMotor = 0.25,	// Guess
		.gravityMv      = 1000,	// Near the tuned kG
		.min            = 0,
		.max            = 3700,	// Guess: skills carries up to 3700
	});

	// Claw on the back, under the winch
	auto& game = sim::game::robot;
	game.claw.right   = odom.getSettings().toolRight;
	game.claw.forward = -w.back;	// Truth, not the config: a wrong toolForward should miss
	game.claw.grab    = [] { return claw.get_voltage() > 0; };
	game.claw.release = [] { return claw.get_voltage() < 0; };
	game.claw.grabForward = -w.back;	// Closes on a piece at the back bumper
	game.claw.sensor  = clawDistance.get_port();

	// activeToggle wheel and colorSensor at the front
	game.spinner.motor   = activeToggle.get_port();
	game.spinner.optical = colorSensor.get_port();
	game.spinner.forward = w.front;

	game.preload = {{.top = sim::game::RED, .bottom = sim::game::YELLOW}};	// Guess: our red/yellow Pin, red up

	// What our drive team drops into the S Loader during Skills, in order: a Pin for Q1, a lone Pin
	// on the empty Q2 goal, then a Cup + Pin on top of it
	using sim::game::PIN, sim::game::CUPPIN;
	sim::game::matchLoads["Skills"] = {.s = {PIN, PIN, CUPPIN}};
}

#pragma endregion
////////////////////////////////////////////////////////////////
