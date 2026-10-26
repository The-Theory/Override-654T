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

#include "pros/llemu.hpp"
#include <vector>



// Brain-screen auton picker: left/right buttons cycle, run() plays the pick
namespace tide::selector {

struct Auton { const char* name; void (*run)(); };

inline std::vector<Auton> autons;
inline int selected = 0;

inline void show() { pros::lcd::print(1, "Auton: < %s >", autons[selected].name); }

inline void cycle(int step) {
	selected = (selected + step + autons.size()) % autons.size();
	show();
}

inline void init(std::vector<Auton> list) {
	autons = list;
	pros::lcd::initialize();
	pros::lcd::register_btn0_cb([] { cycle(-1); });
	pros::lcd::register_btn2_cb([] { cycle( 1); });
	show();
}

inline void run() { autons[selected].run(); }

} // namespace tide::selector
