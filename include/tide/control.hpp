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

#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cstdio>
#include <deque>
#include <functional>
#include <initializer_list>
#include <vector>



namespace tide {

///////////////////////////////////////////////////////////////
#pragma region ButtonNames ////////////////////////////////////
///////////////////////////////////////////////////////////////
// One button, or several held together
struct Input { unsigned int mask; };

// Two inputs controlling one mechanism in opposite directions
struct Pair { Input fwd, rev; };

// Analog stick axis
struct Axis { pros::controller_analog_e_t id; };

/**
 * Adding inputs makes a combo.
 */
inline Input operator+(Input a, Input b) { return {a.mask | b.mask}; }

/**
 * Negating a pair swaps direction.
 * - Avoids requiring port negation.
 */
inline Pair operator-(Pair p) { return {p.rev, p.fwd}; }

// Short names for inputs, pulled in with `using namespace tide::btn;`
namespace btn {
// Buttons
const Input L1      = {1u << pros::E_CONTROLLER_DIGITAL_L1};
const Input L2      = {1u << pros::E_CONTROLLER_DIGITAL_L2};
const Input R1      = {1u << pros::E_CONTROLLER_DIGITAL_R1};
const Input R2      = {1u << pros::E_CONTROLLER_DIGITAL_R2};
const Input UP      = {1u << pros::E_CONTROLLER_DIGITAL_UP};
const Input DOWN    = {1u << pros::E_CONTROLLER_DIGITAL_DOWN};
const Input LEFT    = {1u << pros::E_CONTROLLER_DIGITAL_LEFT};
const Input RIGHT   = {1u << pros::E_CONTROLLER_DIGITAL_RIGHT};
const Input A       = {1u << pros::E_CONTROLLER_DIGITAL_A};
const Input B       = {1u << pros::E_CONTROLLER_DIGITAL_B};
const Input X       = {1u << pros::E_CONTROLLER_DIGITAL_X};
const Input Y       = {1u << pros::E_CONTROLLER_DIGITAL_Y};

// Pairs
const Pair L        = {L1,      L2};
const Pair R        = {R1,      R2};
const Pair DPAD_V   = {UP,      DOWN};
const Pair DPAD_H   = {RIGHT,   LEFT};
const Pair FACE_V   = {Y,       A};
const Pair FACE_H   = {B,       X};

// Axes
const Axis LX       = {pros::E_CONTROLLER_ANALOG_LEFT_X};
const Axis LY       = {pros::E_CONTROLLER_ANALOG_LEFT_Y};
const Axis RX       = {pros::E_CONTROLLER_ANALOG_RIGHT_X};
const Axis RY       = {pros::E_CONTROLLER_ANALOG_RIGHT_Y};
}
#pragma endregion
///////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////
#pragma region ControlClass ///////////////////////////////////
///////////////////////////////////////////////////////////////
// Max voltage used as default for all motors - VEX motors cap inputs above 12V
const int MAX_VOLTAGE = 12000;  // [mV]

// Defined in Triggers below
class When;
class WhenPair;

// One binding with its own toggle and macro state
struct Binding {
	std::function<void(Binding&)> run;
	bool state   = false;   // Toggle state
	bool running = false;   // Macro in progress
	int  step    = 0;       // Next altmacro routine
};

class Control {
public:
	Control(pros::Controller& controller) : ctrl(controller) {}

	/**
	 * Starts a binding on a button or combo.
	 * Example: `when(A + UP)`.
	 */
	When when(Input input);

	/**
	 * Starts a binding on a forward and reverse pair.
	 * Example: `when(R)`.
	 */
	WhenPair when(Pair pair);

	/**
	 * Adds a custom function to run every update.
	 */
	Control& on(std::function<void()> fn) {
		// Saves this binding to run every update()
		return add({}, [fn](Binding&) { fn(); });
	}

	// Down now, and not part of a combo
	bool held(Input in) const {
		return allHeld(in, now) && !shadowed(in, now);
	}

	// Held now but not last cycle
	// Partially letting go of a combo doesn't trigger remaining buttons
	bool pressed(Input in) const {
		return held(in) && !wasHeld(in) && (now & ~was & in.mask);
	}

	// Held last cycle but not now
	// Partially letting go of a combo doesn't trigger remaining buttons
	bool released(Input in) const {
		return wasHeld(in) && !held(in) && (was & ~now & in.mask);
	}

	int axis(Axis a) const { return ctrl.get_analog(a.id); }  // -127 to 127

	/**
	 * Snapshots the controller state, then runs every binding. 
	 * Call each iteration.
	 */
	void update() {
		was = now;  // Save last cycle
		now = 0;
		// Each button represents 1 bit in a pool of 4 bytes (an Integer)
		// Only 12 are in use (20 are kept at 0)
		// Could be used for a second controller with future development
		for (int id = FIRST_BUTTON; id <= LAST_BUTTON; id++) {
			if (ctrl.get_digital((pros::controller_digital_e_t)id)) now |= 1u << id;
		}
		// Run every saved binding
		for (Binding& binding : bindings) binding.run(binding);
	}

private:
	friend class When;
	friend class WhenPair;

	static const int WARN_LINE = 7;  // bottom of screen

	// Digital button ids fitted in a bitmask
	static const int FIRST_BUTTON = pros::E_CONTROLLER_DIGITAL_L1;
	static const int LAST_BUTTON  = pros::E_CONTROLLER_DIGITAL_A;

	/**
	 * Clamps a binding's voltage to what a V5 motor accepts.
	 */
	static int checkVoltage(int mv) {
		const int voltage = std::clamp(mv, -MAX_VOLTAGE, MAX_VOLTAGE);
		if (voltage != mv) {
			// Terminal needs "pros terminal" to show
			printf("Tide: %d mV out of range, clamped to %d\n", mv, voltage);
			pros::lcd::print(WARN_LINE, "Tide: %d mV -> %d", mv, voltage);
		}
		return voltage;
	}

	/**
	 * Runs a function in its own thread in order to avoid disrupting the main program.
	 */
	static void launch(Binding& self, std::function<void()> fn) {
		self.running = true;
		// New thread and run
		pros::Task([&self, fn] {
			// Code here won't disrupt driving, for example
			fn();
			self.running = false;
		});
	}

	/**
	 * Saves a binding and sorts to give combos priority.
	 */
	Control& add(std::initializer_list<Input> inputs, std::function<void(Binding&)> run) {
		for (const Input in : inputs) boundInputs.push_back(in);
		bindings.push_back({run});
		// Returns itself so bindings can chain
		return *this;
	}

	// Every button of the input is down in this snapshot
	static bool allHeld(Input in, unsigned int state) { return (state & in.mask) == in.mask; }

	// Avoids shadow triggers from buttons assigned to a combo
	bool shadowed(Input in, unsigned int state) const {
		for (const Input other : boundInputs) {
			const bool bigger = other.mask != in.mask && (other.mask & in.mask) == in.mask;
			if (bigger && allHeld(other, state)) return true;
		}
		return false;
	}

	// Same as held() but last cycle
	bool wasHeld(Input in) const { return allHeld(in, was) && !shadowed(in, was); }

	pros::Controller& ctrl;
	std::deque<Binding> bindings;   // Deque so macro threads can keep a reference
	std::vector<Input> boundInputs; // Every input a binding uses
	unsigned int now = 0, was = 0;  // One bit per button, this cycle and last
};
#pragma endregion
////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////
#pragma region Triggers ////////////////////////////////////////
////////////////////////////////////////////////////////////////
/**
 * Bindings on a forward and reverse pair.
 * Example: `when(R)`.
 */
class WhenPair {
public:
	WhenPair(Control& control, Pair pair) : ctl(control), fwd(pair.fwd), rev(pair.rev) {}

	/**
	 * Motor control for a forward and reverse button.
	 * Stops on release.
	 * - Useful for intakes.
	 */
	Control& bidir(pros::AbstractMotor& motor, int mv = MAX_VOLTAGE) {
		const int voltage = Control::checkVoltage(mv);
		// Saves this binding to run every update()
		return ctl.add({fwd, rev}, [*this, &motor, voltage](Binding&) {
			const bool wasSpinning = ctl.wasHeld(fwd) || ctl.wasHeld(rev);
			if (ctl.held(fwd)) motor.move_voltage(voltage);       // Spin if forward is held
			else if (ctl.held(rev)) motor.move_voltage(-voltage); // Spin opposite if reverse is held
			else if (wasSpinning) motor.move_voltage(0);          // Stop if neither
		});
	}

private:
	Control& ctl;
	Input fwd, rev;
};

/**
 * Bindings on a button or combo.
 * Can be used for a family of macros, such as using `A` as a modifer to reach several macros:
 * (`A`+`UP`, `A`+`DOWN`, etc).
 */
class When {
public:
	When(Control& control, Input input) : ctl(control), input(input) {}

	/**
	 * Triggers on release instead of press
	 */
	When& onRelease() {
		release = true;
		return *this;
	}

	/**
	 * Runs while button is held
	 * Stops on release
	 */
	Control& hold(pros::AbstractMotor& motor, int mv = MAX_VOLTAGE) {
		// Same input as both halves of the pair
		return WhenPair(ctl, {input, input}).bidir(motor, mv);
	}

	/**
	 * Fires once when triggered
	 * Requires button to be released to be able to trigger again
	 */
	Control& run(std::function<void()> fn) {
		// Saves this binding to run every update()
		return ctl.add({input}, [*this, fn](Binding&) {
			if (triggered()) fn();  // If triggered, run
		});
	}

	/**
	 * New trigger toggles state
	 * Inputted function is run every update with the toggle state
	 * - Useful for pneumatics
	 */
	Control& toggle(std::function<void(bool)> fn) {
		// Saves this binding to run every update()
		return ctl.add({input}, [*this, fn](Binding& self) {
			if (triggered()) self.state = !self.state;  // On trigger, invert state
			fn(self.state);                             // Call function every time, inputting state
		});
	}

	/**
	 * New trigger toggles motor state
	 * Motor never stops
	 */
	Control& toggle(pros::AbstractMotor& motor, int mv = MAX_VOLTAGE) {
		const int voltage = Control::checkVoltage(mv);
		// Reuses toggle() above to handle the on/off state
		return toggle([&motor, voltage](bool state) {
			motor.move_voltage(state ? voltage : 0);  // Lambda to move motor based on state
		});
	}

	/**
	 * Runs fn in its own thread in order to avoid disrupting the main program (op control). 
	 * Triggering a macro while it's running does nothing.
	 */
	Control& macro(std::function<void()> fn) {
		// Saves this binding to run every update()
		return ctl.add({input}, [*this, fn](Binding& self) {
			if (!triggered() || self.running) return;  // Skip unless triggered and free
			Control::launch(self, fn);
		});
	}

	/**
	 * Routines run in order, one per trigger.
	 * Loops back to the first after the last.
	 */
	Control& altmacro(std::initializer_list<std::function<void()>> routines) {
		if (routines.size() == 0) return ctl;
		// Copy the list so it outlives this call
		std::vector<std::function<void()>> list(routines);
		// Saves this binding to run every update()
		return ctl.add({input}, [*this, list](Binding& self) {
			if (!triggered() || self.running) return;  // Skip unless triggered and free
			const int turn = self.step;
			self.step = (turn + 1) % list.size();  // Advance to the next routine
			Control::launch(self, list[turn]);
		});
	}

	/**
	 * Routines run in order, one per trigger.
	 * Loops back to the first after the last.
	 * Takes the routines directly instead of a braced list.
	 */
	template <typename... Fns>
	Control& altmacro(Fns... routines) {
		// Packs the routines into a braced list
		return altmacro({std::function<void()>(routines)...});
	}

private:
	Control& ctl;
	Input input;
	bool release = false;

	// Press or release this cycle, whichever this binding waits for
	bool triggered() const { return release ? ctl.released(input) : ctl.pressed(input); }
};

// Return When and WhenPair, which are defined after Control
inline When Control::when(Input input) { return When(*this, input); }
inline WhenPair Control::when(Pair pair) { return WhenPair(*this, pair); }
#pragma endregion
////////////////////////////////////////////////////////////////

}
