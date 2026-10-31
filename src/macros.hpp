#pragma once

#include "robot.hpp"
#include "tide/screen.hpp"

////////////////////////////////////////////////////////////////
#pragma region Body ////////////////////////////////////////////
////////////////////////////////////////////////////////////////

// Claw control
inline void spinClaw() 	{ claw.move_voltage(12000);}
inline void stopClaw() 	{ claw.move_voltage(0);}
inline void openClaw() 	{ claw.move_voltage(-12000);}

// From scoring height; ends with the winch down
inline void score() {
	openClaw();
	pros::delay(750);
	winch.moveTo(0);
	pros::delay(750);
	if (claw.get_voltage() < 0) stopClaw();	// Driver spinning the claw mid-macro keeps it spinning
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Toggle //////////////////////////////////////////
////////////////////////////////////////////////////////////////

constexpr int TOGGLE_MV   = 12000; 	// max voltage for toggles
constexpr int TOGGLE_NEAR = 50;		// Proximity for toggles

inline bool spinToggle = false;	// Driver override of toggles
inline bool autoFlip   = false;	// Driver's auto flipping; never on before driver control in a match
inline bool flipNow    = false;	// waitForToggle wants the Toggle in reach flipped

// Whether color sensor sees alliance color
inline bool toggleShowsAlliance() {
	const double hue = colorSensor.get_hue();
	if (tide::screen::alliance() == tide::Alliance::RED) return hue < 25 || hue > 335;  // Red
	return hue > 190 && hue < 250;  // Blue
}

// Whether a toggle is close enough
inline bool toggleInReach() {
	const std::int32_t near = colorSensor.get_proximity();
	return near != PROS_ERR && near >= TOGGLE_NEAR;	// Unplugged sensor never spins
}

// Check if an opp. toggle is within range
inline bool toggleNeedsFlip() {
	return toggleInReach() && !toggleShowsAlliance();
}

// Flips a toggle in reach to alliance color while autoFlip or waitForToggle asks
inline void autoToggle() {
	bool lit = false;
	while (true) {
		const bool inReach = toggleInReach();
		if (inReach != lit) colorSensor.set_led_pwm(inReach ? 100 : 0);  // Turn on light when near

		const bool flip = (autoFlip || flipNow) && lit && inReach && !toggleShowsAlliance();
		if (spinToggle || flip) activeToggle.move_voltage(TOGGLE_MV);
		else activeToggle.brake();

		lit = inReach;
		pros::delay(10);
	}
}

// Has autoToggle flip the Toggle in reach and waits for it; false on timeout
inline bool waitForToggle(int timeout = 3000) {
	const std::uint32_t start = pros::millis();
	flipNow = true;
	while (toggleNeedsFlip() && pros::millis() - start < timeout) pros::delay(10);
	flipNow = false;
	return !toggleNeedsFlip();
}

#pragma endregion
////////////////////////////////////////////////////////////////
