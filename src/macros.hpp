#pragma once

#include "robot.hpp"
#include "tide/screen.hpp"
#include <cmath>

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
constexpr double TOGGLE_HUE_SPAN = 25;	// [deg] either side of the alliance hue; narrower and a real face can miss it
constexpr double TOGGLE_MIN_SAT  = 0.25;	// Below this the hue is noise, as in dim light; too high and the alliance face never counts

inline bool spinToggle = false;	// Driver override of toggles
inline bool autoFlip   = false;	// Driver's auto flipping; never on before driver control in a match
inline bool flipNow    = false;	// waitForToggle wants the Toggle in reach flipped

// Whether color sensor clearly sees alliance color; anything unsure counts as not ours, so it flips
inline bool toggleShowsAlliance() {
	const double allianceHue = tide::screen::alliance() == tide::Alliance::RED ? 0 : 220;
	const double off = std::remainder(colorSensor.get_hue() - allianceHue, 360);
	return std::fabs(off) < TOGGLE_HUE_SPAN && colorSensor.get_saturation() >= TOGGLE_MIN_SAT;
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
