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

#include "lemlib/api.hpp"	// IWYU pragma: keep
#include <cmath>

namespace tide {

////////////////////////////////////////////////////////////////
#pragma region Geometry ////////////////////////////////////////
////////////////////////////////////////////////////////////////

// LemLib headings, pulled in with `using namespace tide::dir;`
namespace dir {
constexpr double NORTH = 0;
constexpr double EAST  = 90;
constexpr double SOUTH = 180;
constexpr double WEST  = 270;

constexpr double NORTH_EAST = 45;
constexpr double SOUTH_EAST = 135;
constexpr double SOUTH_WEST = 225;
constexpr double NORTH_WEST = 315;
}

constexpr double TILE = 23.4;	// [in] tile pitch: 140.4" inside the perimeter / 6

// Field position or offset [in]
struct Point { double x, y; };

// Robot-frame offset [in] (+right, +forward) along field axes, for a LemLib heading [deg]
inline Point toField(double right, double forward, double heading) {
	const double h = lemlib::degToRad(heading);
	return {right * std::cos(h) + forward * std::sin(h), -right * std::sin(h) + forward * std::cos(h)};
}

// LemLib heading [deg] that points from one spot at another
inline double bearing(Point from, Point to) { return lemlib::radToDeg(std::atan2(to.x - from.x, to.y - from.y)); }

// Perimeter walls; field frame has +x toward blue station, +y away from audience
enum class Wall { RED, BLUE, AUDIENCE, FAR };

#pragma endregion
////////////////////////////////////////////////////////////////

}
