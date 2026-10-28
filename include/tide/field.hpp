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

////////////////////////////////////////////////////////////////
#pragma region Positions ///////////////////////////////////////
////////////////////////////////////////////////////////////////

// Override field, red side; blue runs the same code from the spot rotated 180°
// Pulled in with `using namespace tide::field;`, map in docs/field-map.svg
namespace field {

namespace goal {
constexpr Point Q1_ALLIANCE = {-47.1, -23.5};
constexpr Point Q1_NEUTRAL  = {-47.1,  23.6};
constexpr Point Q2_ALLIANCE = {-23.5, -47.1};
constexpr Point Q2_NEUTRAL  = { 23.6, -47.1};
constexpr Point MIDFIELD    = {  0,     0  };
}

namespace cuppin {
constexpr Point RED_WALL_S      = {-68.6, -23.5};
constexpr Point RED_WALL_N      = {-68.6,  23.6};
constexpr Point AUDIENCE_WALL_W = {-23.5, -68.6};
constexpr Point AUDIENCE_WALL_E = { 23.6, -68.6};
constexpr Point DIAGONAL_INNER  = {-23.5, -23.5};
constexpr Point DIAGONAL_OUTER  = {-47.1, -47.1};
constexpr Point MIDFIELD_W      = {-23.5,   0  };
constexpr Point MIDFIELD_S      = {  0,   -23.5};
}

namespace cross {
constexpr Point Q1_INNER = {-23.5,  23.6};
constexpr Point Q1_OUTER = {-47.1,  47.1};
constexpr Point Q2_INNER = { 23.6, -23.5};
constexpr Point Q2_OUTER = { 47.1, -47.1};
}

// Wall-mount centers
// Ram with pushUntilStall
namespace toggle {
constexpr Point Q1 = {-70.2,   0  };
constexpr Point Q2 = {  0,   -70.2};
}

}

#pragma endregion
////////////////////////////////////////////////////////////////

}
