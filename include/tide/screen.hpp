////////////////////////////////////////////////////////////////
// Tide - Controller binding library for PROS				  //
// Built for 654T - Tsunami, VEX Override 2026-2027			  //
//															  //
// By Theo Hallgren											  //
// Documentation in docs/tide.md							  //
//															  //
// Source available on: github.com/The-Theory/Override-654T	  //
// under GPL-3.0 license									  //
////////////////////////////////////////////////////////////////


#pragma once

#include "pros/distance.hpp"
#include "pros/error.h"
#include "pros/motor_group.hpp"
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>



namespace tide {

enum class Alliance { RED, BLUE };

// Brain touchscreen: Auton (alliance, options, auton), Tune (tuners + result), Info (live readings)
namespace screen {

////////////////////////////////////////////////////////////////
#pragma region Entries /////////////////////////////////////////
////////////////////////////////////////////////////////////////

struct Auton {
	const char* name;
	void (*run)(Alliance);
};

// Returns text to show when it finishes, e.g. tuned gains
struct Tuner {
	const char* name;
	std::string (*run)();
};

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region State ///////////////////////////////////////////
////////////////////////////////////////////////////////////////

namespace detail {

inline std::vector<Auton> autons;
inline std::vector<Tuner> tuners;
inline std::vector<const char*> options;
inline std::function<std::string()> info;

// Written by touch callbacks (LVGL task), read by auton/driver tasks; plain ints are fine
inline int autonIdx = 0;
inline int tunerIdx = 0;
inline int pageIdx  = 0;
inline Alliance side = Alliance::RED;
inline std::vector<char> optionOn;

// Tuners run outside the LVGL task, so they hand text over through a buffer the refresh timer picks up
inline char report[512] = "";
inline std::atomic<int> reportVersion = 0;

}

#pragma endregion
////////////////////////////////////////////////////////////////

} // namespace screen
} // namespace tide

// LVGL pages, separate so off-brain builds can swap them out
#include "tide/screen_ui.hpp"

namespace tide {
namespace screen {

////////////////////////////////////////////////////////////////
#pragma region API /////////////////////////////////////////////
////////////////////////////////////////////////////////////////

/**
 * Builds the screen. Call once in initialize().
 * - info: text for the Info page as "Key\tvalues" lines, refreshed 10x a second while it's open
 * - options: on/off toggles under the alliance button (room for 2), read with option()
 */
inline void init(std::vector<Auton> autons, std::vector<Tuner> tuners, std::function<std::string()> info = {},
				 std::vector<const char*> options = {}) {
	using namespace detail;
	detail::autons  = autons;
	detail::tuners  = tuners;
	detail::info    = info;
	detail::options = options;
	optionOn.assign(options.size(), false);

	build();
}

inline Alliance alliance() { return detail::side; }

// Whether the named option toggle is on; false for names not passed to init()
inline bool option(const char* name) {
	for (int i = 0; i < (int)detail::options.size(); i++)
		if (std::strcmp(detail::options[i], name) == 0) return detail::optionOn[i];
	return false;
}

// Plays the selected auton for the selected alliance
inline void runAuton() {
	if (!detail::autons.empty()) detail::autons[detail::autonIdx].run(detail::side);
}

// Runs the selected tuner and shows what it returns on the Tune page
inline void runTuner() {
	if (detail::tuners.empty()) return;
	const std::string result = detail::tuners[detail::tunerIdx].run();
	std::snprintf(detail::report, sizeof(detail::report), "%s", result.c_str());
	detail::reportVersion++;
}

// For info text: a distance reading in inches, or "--" with nothing in range
inline std::string inches(pros::Distance& sensor) {
	const std::int32_t mm = sensor.get();
	if (mm == PROS_ERR || mm >= 9999) return "--";
	char str[16];
	std::snprintf(str, sizeof(str), "%.1f", mm / 25.4);
	return str;
}

// For info text: the hottest motor in a group [°C]
inline double hottest(pros::MotorGroup& motors) {
	const std::vector<double> temps = motors.get_temperature_all();
	return temps.empty() ? 0 : *std::max_element(temps.begin(), temps.end());
}

#pragma endregion
////////////////////////////////////////////////////////////////

} // namespace screen
} // namespace tide
