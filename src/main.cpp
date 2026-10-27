////////////////////////////////////////////////////////////////
// Codebase of 654T - Tsunami								  //
// Lake Travis High School, Texas, United States			  //
// VEX Override 2026-2027									  //
//															  //
// By Theo Hallgren, Ryan Koontz, and Sebastian Ditsch 		  //
// Using LemLib by Liam Teale								  //
// 															  //
// Source available on: github.com/The-Theory/Override-654T	  //
// under GPL-3.0 license									  //
////////////////////////////////////////////////////////////////

#include "robot.hpp"
#include "tide/control.hpp"

using namespace tide::btn;

////////////////////////////////////////////////////////////////

void initialize() {
	tide::screen::init(autons, tuners, robotInfo, {"Nanny"});
	initRobot();
}

void disabled() {}
void competition_initialize() {}
void autonomous() { tide::screen::runAuton(); }

////////////////////////////////////////////////////////////////

void opcontrol() {
	tide::Control ctl(controller);
	ctl.when(L).bidir(winchPID)
		.when(R).bidir(claw)
		.when(X).macro(score)
		.when(LEFT).run(autonomous)
		.when(RIGHT).run(tide::screen::runTuner);

	while (true) {
		ctl.update();
		chassis.curvature(ctl.axis(LY), ctl.axis(RX));
		pros::delay(25);
	}
}
