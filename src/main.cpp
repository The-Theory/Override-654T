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
#include <algorithm>
#include <cstdio>

using namespace tide::btn;



////////////////////////////////////////////////////////////////
#pragma region ScreenInfo //////////////////////////////////////
////////////////////////////////////////////////////////////////

std::string inches(pros::Distance& sensor) {
	const std::int32_t mm = sensor.get();
	if (mm == PROS_ERR || mm >= 9999) return "--";
	char text[16];
	std::snprintf(text, sizeof(text), "%.1f", mm / 25.4);
	return text;
}

double hottest(pros::MotorGroup& motors) {
	const std::vector<double> temps = motors.get_temperature_all();
	return temps.empty() ? 0 : *std::max_element(temps.begin(), temps.end());
}

// INFO tab: what's needed to calibrate and diagnose, refreshed 10x a second
std::string robotInfo() {
	const lemlib::Pose pose = chassis.getPose();
	char text[512];
	std::snprintf(text, sizeof(text),
		"Pose        x %.1f    y %.1f    heading %.1f°\n"
		"Distance    left %s    right %s    middle %s  [in]\n"
		"Winch       %.0f°\n"
		"Claw pivot  %.0f°    sensor absolute %.1f°\n"
		"Temps       drive %.0f    winch %.0f    claw %.0f    pivot %.0f  [°C]",
		pose.x, pose.y, pose.theta,
		inches(leftDistance).c_str(), inches(rightDistance).c_str(), inches(middleDistance).c_str(),
		winchPID.position(),
		clawPivotPID.position(), clawEncoder.get_angle() / 100.0,
		std::max(hottest(leftMotors), hottest(rightMotors)), hottest(winch), claw.get_temperature(), clawPivot.get_temperature());
	return text;
}

#pragma endregion
////////////////////////////////////////////////////////////////

/**
 * Initalization triggered upon execution.
 */
void initialize() {
	tide::screen::init(autons, tuners, robotInfo, "654T TSUNAMI");
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
void autonomous() { tide::screen::runAuton(); }

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
		.when(RIGHT).run(tide::screen::runTuner)
		.when(Y).macro(tareClaw);

	while (true) {
		// Refresh
		tide.update();
		pros::delay(25);

		// Move
		chassis.curvature(tide.axis(LY), tide.axis(RX));
	}
}
