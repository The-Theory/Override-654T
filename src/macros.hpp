#pragma once

#include "robot.hpp"
#include "tide/screen.hpp"

constexpr int TOGGLE_MV   = 8000;	// Slow enough to stop on the face, not past it
constexpr int TOGGLE_NEAR = 100;	// colorSensor proximity (0 far - 255 touching) that counts as a Toggle in reach

// Claw control
inline void intakeClaw() 	{ claw.move_voltage(12000); }
inline void stopClaw() 	{ claw.move_voltage(0); 	}
inline void outtakeClaw() 	{ claw.move_voltage(-12000);}

// From scoring height; ends with the winch down
inline void score() {
	outtakeClaw();
	winch.moveTo(0);
	pros::delay(750);
	stopClaw();
}

// Whether colorSensor sees our alliance's face; hue windows [deg] tuned with the Info page
inline bool toggleShowsAlliance() {
	const double hue = colorSensor.get_hue();
	if (tide::screen::alliance() == tide::Alliance::RED) return hue < 25 || hue > 335;	// Red wraps past 0
	return hue > 190 && hue < 250;
}

// Spins the Toggle until it shows the alliance picked on the brain; false on timeout or no Toggle in reach
inline bool toggleToAlliance(int timeout = 3000) {
	if (colorSensor.get_proximity() < TOGGLE_NEAR) return false;
	const std::uint32_t start = pros::millis();
	activeToggle.move_voltage(TOGGLE_MV);
	while (!toggleShowsAlliance() && pros::millis() - start < timeout) pros::delay(10);
	activeToggle.move_voltage(0);
	return toggleShowsAlliance();
}
