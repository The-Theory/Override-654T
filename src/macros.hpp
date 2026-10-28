#pragma once

#include "robot.hpp"
#include "tide/screen.hpp"

////////////////////////////////////////////////////////////////
#pragma region Body ////////////////////////////////////////////
////////////////////////////////////////////////////////////////

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

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Toggle //////////////////////////////////////////
////////////////////////////////////////////////////////////////

constexpr int TOGGLE_MV   = 12000; 	// Voltage for toggles
constexpr int TOGGLE_NEAR = 50;		// Proximity for toggles: 0 far, 255 touching; keep below the lowest Info "near" while one is in reach

// Whether colorSensor sees alliance color
inline bool toggleShowsAlliance() {
	const double hue = colorSensor.get_hue();
	if (tide::screen::alliance() == tide::Alliance::RED) return hue < 25 || hue > 335;  // Red
	return hue > 190 && hue < 250;  // Blue
}

// Check if an opp. toggle is within range
inline bool toggleNeedsFlip() {
	const std::int32_t near = colorSensor.get_proximity();
	return near != PROS_ERR && near >= TOGGLE_NEAR && !toggleShowsAlliance();	// Unplugged sensor never spins
}

// Runs all match from initialize(): flips any Toggle in reach to alliance color
inline void autoToggle() {
	while (true) {
		if (toggleNeedsFlip()) activeToggle.move_voltage(TOGGLE_MV);
		else activeToggle.brake();	// move_voltage(0) coasts past the face
		pros::delay(10);
	}
}

// For autons: waits until autoToggle is done with the Toggle in reach; false on timeout
inline bool waitForToggle(int timeout = 3000) {
	const std::uint32_t start = pros::millis();
	while (toggleNeedsFlip() && pros::millis() - start < timeout) pros::delay(10);
	return !toggleNeedsFlip();
}

#pragma endregion
////////////////////////////////////////////////////////////////

