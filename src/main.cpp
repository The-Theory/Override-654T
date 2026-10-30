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

void disabled() {}
void competition_initialize() {}
void autonomous() {
	autoFlip = false;	// Only waitForToggle flips in an auton; the Y binding restores it after a driver-run test
	tide::screen::runAuton();
}

////////////////////////////////////////////////////////////////

void opcontrol() {
	// Without field control the program starts here, so a Toggle set up for an auton test waits for Y
	const bool flipFromStart = pros::competition::is_connected();

	tide::Control ctlr(controller);
	ctlr.when(L).bidir(winch)
		.when(R1).toggle(claw)
		.when(X).macro(score)
		.when(LEFT).run(autonomous)
		.when(RIGHT).run(tide::screen::runTuner)
		.when(Y).toggle([=](bool pressed) { autoFlip = pressed != flipFromStart; })	// Y switches auto flipping
		.on([&] { spinToggle = ctlr.held(R2); });

	while (true) {
		ctlr.update();
		chassis.curvature(ctlr.axis(LY), ctlr.axis(RX));
	}
}
