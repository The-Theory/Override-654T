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

// Competition entry points. Hardware & tuning: robot.cpp, macros: macros.cpp, routes: autons.cpp

#include "robot.hpp"
#include "tide/control.hpp"

using namespace tide::btn;



/**
 * Initalization triggered upon execution.
 */
void initialize() {
	tide::selector::init(autons);
	// Here, not opcontrol: field auton runs before opcontrol, so motors would still coast
	claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.calibrate();
	clawPivotPID.start();
	winchPID.tare();
	winchPID.start();
}

/**
 * Code lock during disabled state via Field Management System or
 * the VEX Competition Switch.
 */
void disabled() {}

/**
 * Run after initalize() before match starts.
 * Allows selection of specific autons.
 */
void competition_initialize() {}

/**
 * Runs the autonomous code via Field Management System or
 * the VEX Competition Switch. May be called manually for testing.
 */
void autonomous() { tide::selector::run(); }

/**
 * Runs the  control code via Field Management System or
 * the VEX Competition Switch, or after initialize() when
 * not in competition mode.
 */
void opcontrol() {
	// Def controls
	tide::Control tide(controller);
	tide.when(R).bidir(intakeMotor)
		.when(R1).onlyIf(isWinchDown).macro(clawIntakeMacro)
		.when(L).bidir(winchPID)
		.when(R).bidir(claw)
		.when(A).altmacro(pickupMacroPart1, [] { pickupMacroPart2(); })
		.when(X).altmacro(clawPivotUp, scoringMacro)
		.when(B).run([] { clawPivotPID.release(); })
		.when(LEFT).run(autonomous)
		.when(Y).macro(tareClaw);

	while (true) {
		// Refresh
		tide.update();
		pros::delay(25);

		// Move
		chassis.curvature(tide.axis(LY), tide.axis(RX));
	}
}
