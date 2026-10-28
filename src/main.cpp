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

#include "macros.hpp"
#include "tide/control.hpp"
#include "tide/screen.hpp"

using namespace tide::btn;

////////////////////////////////////////////////////////////////

void initScreen();	// autons.cpp

void initialize() {
	initScreen();
	initRobot();
}

void disabled() {}
void competition_initialize() {}
void autonomous() { tide::screen::runAuton(); }

////////////////////////////////////////////////////////////////

void opcontrol() {
	tide::Control ctlr(controller);
	ctlr.when(L).bidir(winchPID)
		.when(R1).toggle(claw)
		.when(R2).toggle(activeToggle)
		.when(X).macro(score)
		.when(LEFT).run(autonomous)
		.when(RIGHT).run(tide::screen::runTuner);

	while (true) {
		ctlr.update();
		chassis.curvature(ctlr.axis(LY), ctlr.axis(RX));
	}
}
