////////////////////////////////////////////////////////////////
// TsuControl - controller bindings for 654T - Tsunami		  //
//															  //
// Docs and examples in include/tsu/README.md				  //
////////////////////////////////////////////////////////////////


#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cstdio>
#include <functional>
#include <initializer_list>
#include <vector>



namespace tsu {

////////////////////////////////////////////////////////////////
#pragma region ButtonNames /////////////////////////////////////
////////////////////////////////////////////////////////////////
const int MAX_VOLTAGE = 12000;  // [mV]

// One physical button
struct Button {
	pros::controller_digital_e_t id;
};

// Two buttons driving one mechanism in opposite directions
struct Pair {
	Button fwd, rev;
};

// An analog stick axis
struct Axis {
	pros::controller_analog_e_t id;
};

/**
 * Negating a pair swaps which half drives forward, so -R is R2 forward.
 * Avoids requiring the use of port negation.
 */
inline Pair operator-(Pair p) { return {p.rev, p.fwd}; }

// Short names, pulled in with `using namespace tsu::btn;`
namespace btn {

const Button L1    = {pros::E_CONTROLLER_DIGITAL_L1};
const Button L2    = {pros::E_CONTROLLER_DIGITAL_L2};
const Button R1    = {pros::E_CONTROLLER_DIGITAL_R1};
const Button R2    = {pros::E_CONTROLLER_DIGITAL_R2};
const Button UP    = {pros::E_CONTROLLER_DIGITAL_UP};
const Button DOWN  = {pros::E_CONTROLLER_DIGITAL_DOWN};
const Button LEFT  = {pros::E_CONTROLLER_DIGITAL_LEFT};
const Button RIGHT = {pros::E_CONTROLLER_DIGITAL_RIGHT};
const Button A     = {pros::E_CONTROLLER_DIGITAL_A};
const Button B     = {pros::E_CONTROLLER_DIGITAL_B};
const Button X     = {pros::E_CONTROLLER_DIGITAL_X};
const Button Y     = {pros::E_CONTROLLER_DIGITAL_Y};

// Pairs, first name drives forward
const Pair L      = {L1,    L2};
const Pair R      = {R1,    R2};
const Pair DPAD_V = {UP,    DOWN};
const Pair DPAD_H = {RIGHT, LEFT};
const Pair FACE_V = {Y,     A};
const Pair FACE_H = {B,     X};

const Axis LX = {pros::E_CONTROLLER_ANALOG_LEFT_X};
const Axis LY = {pros::E_CONTROLLER_ANALOG_LEFT_Y};
const Axis RX = {pros::E_CONTROLLER_ANALOG_RIGHT_X};
const Axis RY = {pros::E_CONTROLLER_ANALOG_RIGHT_Y};

}
#pragma endregion
////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////
#pragma region ControlClass /////////////////////////////////////
////////////////////////////////////////////////////////////////
class TsuControl {
public:
	TsuControl(pros::Controller& controller) : ctrl(controller) {}

	/**
	 * Full voltage while p.fwd is held, reversed while p.rev is held, and
	 * stopped otherwise. Forward wins if both are down.
	 */
	TsuControl& bidir(pros::AbstractMotor& motor, Pair p, int mv = MAX_VOLTAGE) {
		const int voltage = checkVoltage(mv);
		return on([this, &motor, p, voltage] {
			int out = 0;
			if (held(p.fwd)) out = voltage;
			else if (held(p.rev)) out = -voltage;
			motor.move_voltage(out);
		});
	}

	/**
	 * Runs while the button is held, stops on release.
	 */
	TsuControl& hold(pros::AbstractMotor& motor, Button b, int mv = MAX_VOLTAGE) {
		return bidir(motor, {b, b}, mv);
	}

	/**
	 * Each new press toggles the state.
	 */
	TsuControl& toggle(Button b, std::function<void(bool)> fn) {
		return on([this, b, fn] {
			if (pressed(b)) toggleState[idx(b)] = !toggleState[idx(b)];
			fn(toggleState[idx(b)]);
		});
	}

	/**
	 * Motor spins while toggled on.
	 */
	TsuControl& toggle(pros::AbstractMotor& motor, Button b, int mv = MAX_VOLTAGE) {
		const int voltage = checkVoltage(mv);
		return toggle(b, [&motor, voltage](bool state) {
			motor.move_voltage(state ? voltage : 0);
		});
	}

	/**
	 * Fires once on the button press.
	 */
	TsuControl& press(Button b, std::function<void()> fn) {
		return on([this, b, fn] { if (pressed(b)) fn(); });
	}

	/**
	 * Fires once on the button release.
	 */
	TsuControl& release(Button b, std::function<void()> fn) {
		return on([this, b, fn] { if (released(b)) fn(); });
	}

	/**
	 * Fires once each time every button lines up as held, and rearms as soon
	 * as any one of them is let go.
	 */
	TsuControl& combo(std::initializer_list<Button> buttons, std::function<void()> fn) {
		unsigned int mask = 0;
		for (const Button b : buttons) mask |= bit(b);
		return on([this, mask, fn] {
			if ((now & mask) == mask && (was & mask) != mask) fn();
		});
	}

	/**
	 * Runs fn in its own thread so a macro may call pros::delay() without
	 * pausing opcontrol. Clone macro calls during a run *are* ignored.
	 */
	TsuControl& macro(Button b, std::function<void()> fn) {
		return press(b, [this, b, fn] {
			if (macroRunning[idx(b)]) return;
			macroRunning[idx(b)] = true;
			pros::Task([this, b, fn] {
				fn();
				macroRunning[idx(b)] = false;
			});
		});
	}

	/**
	 * Adds a job to the binding list.
	 */
	TsuControl& on(std::function<void()> fn) {
		bindings.push_back(fn);
		return *this;
	}

	// Button states
	bool held(Button b)     const { return now & bit(b); }
	bool pressed(Button b)  const { return (now & ~was) & bit(b); }
	bool released(Button b) const { return (was & ~now) & bit(b); }
	int  axis(Axis a)       const { return ctrl.get_analog(a.id); }
	bool toggled(Button b)  const { return toggleState[idx(b)]; }

	/**
	 * Snapshots the controller state, then runs every binding
	 */
	void update() {
		was = now;
		now = 0;
		for (int id = FIRST_BUTTON; id <= LAST_BUTTON; id++) {
			if (ctrl.get_digital((pros::controller_digital_e_t)id)) now |= 1u << id;
		}
		for (auto& binding : bindings) binding();
	}

private:
	static const int WARN_LINE = 7;  // bottom of screen

	// Digital button ids fitted in a bitmask
	static const int FIRST_BUTTON = pros::E_CONTROLLER_DIGITAL_L1;
	static const int LAST_BUTTON  = pros::E_CONTROLLER_DIGITAL_A;

	// Define size of button array. PWR buttons is max, and len(arr)=n+1
	// due to 0-based indexing
	static const int BUTTON_SLOTS = pros::E_CONTROLLER_DIGITAL_POWER + 1;

	/**
	 * Clamps a binding's voltage to what a V5 motor accepts.
	 */
	static int checkVoltage(int mv) {
		const int voltage = std::clamp(mv, -MAX_VOLTAGE, MAX_VOLTAGE);
		if (voltage != mv) {
			// Terminal needs "pros terminal" to show
			printf("TsuControl: %d mV out of range, clamped to %d\n", mv, voltage);
			pros::lcd::print(WARN_LINE, "TsuControl: %d mV -> %d", mv, voltage);
		}
		return voltage;
	}

	static int idx(Button b) { return (int)b.id; }
	static unsigned int bit(Button b) { return 1u << (int)b.id; }

	pros::Controller& ctrl;
	std::vector<std::function<void()>> bindings;
	unsigned int now = 0, was = 0;

	// Per-button/-macro state, ran by update()
	bool toggleState[BUTTON_SLOTS] = {};
	bool macroRunning[BUTTON_SLOTS] = {};
};
#pragma endregion
////////////////////////////////////////////////////////////////

}
