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

// Entry points and driver binds. Hardware: robot.cpp, macros: macros.cpp,
// autons & tuners: autons.cpp, INFO tab: screen.cpp

#include "robot.hpp"
#include "tide/control.hpp"

using namespace tide::btn;



void initialize() {
	tide::screen::init(autons, tuners, robotInfo);	// First, so the screen is up during IMU calibration
	initRobot();
}

void disabled() {}

void competition_initialize() {}

// Auton is picked on the brain screen
void autonomous() { tide::screen::runAuton(); }

void opcontrol() {
	tide::Control ctl(controller);
	ctl.when(R).bidir(intake)
		.when(R1).onlyIf(isWinchDown).macro(lowerToIntake)
		.when(L).bidir(winchPID)
		.when(R).bidir(claw)
		.when(A).altmacro(raiseForPickup, [] { pickup(); })
		.when(X).altmacro(clawPivotUp, score)
		.when(B).run([] { clawPivotPID.release(); })
		.when(LEFT).run(autonomous)
		.when(RIGHT).run(tide::screen::runTuner)
		.when(Y).macro(tareClawPivot);

	while (true) {
		ctl.update();
		chassis.curvature(ctl.axis(LY), ctl.axis(RX));
		pros::delay(25);
	}
}
