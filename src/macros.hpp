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

// Whether colorSensor sees alliance color
inline bool toggleShowsAlliance() {
	const double hue = colorSensor.get_hue();
	if (tide::screen::alliance() == tide::Alliance::RED) return hue < 25 || hue > 335;	// Red wraps past 0
	return hue > 190 && hue < 250;
}

// Toggle in reach that isn't alliance color yet
inline bool toggleNeedsFlip() {
	const std::int32_t near = colorSensor.get_proximity();
	return near != PROS_ERR && near >= TOGGLE_NEAR && !toggleShowsAlliance();	// Unplugged sensor never spins
}

// Runs all match from initialize(): flips any Toggle in reach to alliance color
inline void autoToggle() {
	while (true) {
		activeToggle.move_voltage(toggleNeedsFlip() ? TOGGLE_MV : 0);
		pros::delay(10);
	}
}

// For autons: waits until autoToggle is done with the Toggle in reach; false on timeout
inline bool waitForToggle(int timeout = 3000) {
	const std::uint32_t start = pros::millis();
	while (toggleNeedsFlip() && pros::millis() - start < timeout) pros::delay(10);
	return !toggleNeedsFlip();
}
