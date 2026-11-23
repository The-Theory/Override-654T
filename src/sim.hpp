#pragma once

// 654T in the desktop sim (tools/sim); the brain build never includes this

#include "macros.hpp"
#include "game.hpp"
#include "world.hpp"

////////////////////////////////////////////////////////////////
#pragma region SimRobot ////////////////////////////////////////
////////////////////////////////////////////////////////////////

// What the robot physically is; drivetrain and tracking wheel come from robot.hpp unless set here
inline void simRobot(sim::World& w) {
	w.width = 13.6;			// [in]
	w.back  = 5.2;				// Toggle wheel end
	w.front = 15.718 - w.back;	// Claw end; 15.718" long overall

	for (const tide::DistanceMount& m : odom.getMounts()) w.mounts[m.sensor.get_port()] = {m.right, m.forward, m.facing};

	w.liftTau = 0.1;	// Guess: a raised cascade slows the drive's response this much

	// The sim's own FF runs (sim --tune "FF down" / "FF up"), until robot.hpp has the real robot's
	tide::OdomSettings& s = odom.getSettings();
	if (!s.lateral.measured()) {
		s.lateral       = {1205, 166.49, 41.66};
		s.lateralLoaded = {1205, 166.49, 58.24};
		s.angular       = {1205, 16.59, 4.15};
		s.angularLoaded = {1205, 16.59, 5.80};
	}

	w.joints.push_back({
		.ports          = winchMotors.get_port_all(),
		.sensor         = winchEncoder.get_port(),
		.sensorPerMotor = 0.25,	// Guess
		.gravityMv      = 1000,	// Near the tuned kG
		.min            = 0,
		.max            = 3700,	// Guess: skills carries up to 3700
	});

	// Claw on the front, under the winch
	auto& game = sim::game::robot;
	game.claw.right   = odom.getSettings().toolRight;
	game.claw.forward = w.front;	// Truth, not the config: a wrong toolForward should miss
	game.claw.grab    = [] { return claw.get_voltage() > 0; };
	game.claw.release = [] { return claw.get_voltage() < 0; };
	game.claw.grabForward = w.front;	// Closes on a piece at the front bumper
	game.claw.sensor  = clawDistance.get_port();

	// activeToggle wheel and colorSensor at the back
	game.spinner.motor   = activeToggle.get_port();
	game.spinner.optical = colorSensor.get_port();
	game.spinner.forward = -w.back;

	game.recorder = &recorder;
	game.preload = {{.top = sim::game::RED, .bottom = sim::game::YELLOW}};	// Guess: our red/yellow Pin, red up

	// What our drive team drops into the S Loader during Skills, in order: a Pin for Q1, a lone Pin
	// on the empty Q2 goal, then a Cup + Pin on top of it
	using sim::game::PIN, sim::game::CUPPIN;
	sim::game::matchLoads["Skills"] = {.s = {PIN, PIN, CUPPIN}};
	sim::game::matchLoads["Replay"] = sim::game::matchLoads["Skills"];	// Replays a Skills run
}

#pragma endregion
////////////////////////////////////////////////////////////////
