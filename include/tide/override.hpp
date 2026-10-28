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

#include "tide/field.hpp"

// VEX Override 2026-2027 game element positions
namespace tide {

////////////////////////////////////////////////////////////////
#pragma region Positions ///////////////////////////////////////
////////////////////////////////////////////////////////////////

// Red side; blue runs the same code from the spot rotated 180°
// Pulled in with `using namespace tide::override;`, map in docs/field-map.svg
namespace override {

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
