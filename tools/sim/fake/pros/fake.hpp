#pragma once

#include "world.hpp"
#include "pros/error.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <initializer_list>
#include <utility>
#include <vector>

// The slice of PROS this codebase uses, backed by sim::world()
namespace pros {

////////////////////////////////////////////////////////////////
#pragma region RTOS ////////////////////////////////////////////
////////////////////////////////////////////////////////////////

inline void delay(std::uint32_t ms) { sim::world().sched.delay(ms); }
inline std::uint32_t millis() { return sim::world().sched.now; }

class Task {
public:
	template <class F>
	Task(F&& fn, const char* = "") { sim::world().sched.spawn(std::function<void()>(std::forward<F>(fn))); }

	static void delay(std::uint32_t ms) { pros::delay(ms); }
	static void delay_until(std::uint32_t* prev, std::uint32_t delta) {
		*prev += delta;
		const std::uint32_t now = millis();
		pros::delay(*prev > now ? *prev - now : 0);
	}
};

// Only one task runs at a time
struct Mutex {
	bool take(std::uint32_t = 0) { return true; }
	bool give() { return true; }
	void lock() {}
	void unlock() {}
};

namespace competition {
inline std::uint8_t is_disabled() { return false; }
inline std::uint8_t is_autonomous() { return true; }
inline std::uint8_t is_connected() { return false; }
}

namespace lcd {
inline bool initialize() { return true; }
template <class... Args>
bool print(std::int16_t line, const char* fmt, Args... args) {
	std::printf("lcd %d: ", line);
	std::printf(fmt, args...);
	std::printf("\n");
	return true;
}
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Motors //////////////////////////////////////////
////////////////////////////////////////////////////////////////

enum motor_brake_mode_e { E_MOTOR_BRAKE_COAST, E_MOTOR_BRAKE_BRAKE, E_MOTOR_BRAKE_HOLD };

enum class MotorGears { red, green, blue };
using MotorGearset = MotorGears;

// Motor and MotorGroup share everything, so it all lives here
class AbstractMotor {
public:
	AbstractMotor(std::vector<std::int8_t> ports, MotorGears gears) : ports(ports) {
		const double rpm = gears == MotorGears::blue ? 600 : gears == MotorGears::red ? 100 : 200;
		for (int p : ports) sim::world().motor(p).cartridge = rpm;
	}

	std::int32_t move_voltage(std::int32_t mv) {
		for (int p : ports) sim::world().motor(p).mv = std::clamp<std::int32_t>(mv, -12000, 12000);
		return 1;
	}
	std::int32_t move(std::int32_t power) { return move_voltage(power * 12000 / 127); }
	std::int32_t brake() { return move_voltage(0); }

	std::int32_t set_brake_mode(motor_brake_mode_e mode, std::uint8_t = 0) { return set_brake_mode_all(mode); }
	std::int32_t set_brake_mode_all(motor_brake_mode_e mode) {
		for (int p : ports) sim::world().motor(p).brake = (sim::Brake)mode;
		return 1;
	}

	double get_position(std::uint8_t i = 0) const { return state(i).degrees; }
	double get_actual_velocity(std::uint8_t i = 0) const { return state(i).rpm; }
	double get_temperature(std::uint8_t = 0) const { return 30; }
	std::vector<double> get_temperature_all() const { return std::vector<double>(ports.size(), 30); }
	std::int32_t get_voltage(std::uint8_t i = 0) const { return state(i).mv; }
	std::int32_t tare_position(std::uint8_t i = 0) { sim::world().motor(ports[i]).degrees = 0; return 1; }
	std::vector<std::int8_t> get_port_all() const { return ports; }
	std::int8_t get_port(std::uint8_t i = 0) const { return ports[i]; }

private:
	std::vector<std::int8_t> ports;
	sim::Motor& state(std::uint8_t i) const { return sim::world().motor(ports[i]); }
};

struct Motor : AbstractMotor {
	Motor(std::int8_t port, MotorGears gears = MotorGears::green) : AbstractMotor({port}, gears) {}
};

struct MotorGroup : AbstractMotor {
	MotorGroup(std::initializer_list<std::int8_t> ports, MotorGears gears = MotorGears::green) : AbstractMotor(ports, gears) {}
};

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Sensors /////////////////////////////////////////
////////////////////////////////////////////////////////////////

// Port signs pick reversal on the brain; the sim reports what the robot code expects, so they're ignored

class Rotation {
public:
	Rotation(std::int8_t port) : port(port) {}
	std::int32_t get_position() const { return std::lround(state().centideg); }
	std::int32_t get_velocity() const { return std::lround(state().velocity); }
	std::int32_t get_angle() const {
		const std::int32_t a = get_position() % 36000;
		return a < 0 ? a + 36000 : a;
	}
	std::int32_t reset_position() { state().centideg = 0; return 1; }
	std::int32_t set_position(std::int32_t centideg) { state().centideg = centideg; return 1; }
	std::int32_t reset() { return reset_position(); }
	std::int8_t get_port() const { return port; }

private:
	std::int8_t port;
	sim::Rotation& state() const { return sim::world().rotation(port); }
};

class Imu {
public:
	Imu(std::uint8_t) {}
	std::int32_t reset(bool = false) { return 1; }
	bool is_calibrating() const { return false; }
	double get_rotation() const { return sim::world().imu() + offset; }
	double get_heading() const {
		const double h = std::fmod(get_rotation(), 360);
		return h < 0 ? h + 360 : h;
	}
	std::int32_t set_rotation(double deg) { offset += deg - get_rotation(); return 1; }
	std::int32_t set_heading(double deg) { return set_rotation(deg); }
	std::int32_t tare_rotation() { return set_rotation(0); }
	std::int32_t tare_heading() { return set_rotation(0); }

private:
	double offset = 0;
};

class Distance {
public:
	Distance(std::uint8_t port) : port(port) {}
	std::int32_t get() const { return sim::world().distance(port); }
	std::int32_t get_distance() const { return get(); }
	std::int32_t get_confidence() const { return get() == 9999 ? 0 : 63; }
	std::uint8_t get_port() const { return port; }

private:
	std::uint8_t port;
};

// ponytail: sees nothing until game elements and Toggles exist
class Optical {
public:
	Optical(std::uint8_t) {}
	double get_hue() const { return 0; }
	std::int32_t get_proximity() const { return 0; }
	std::int32_t set_led_pwm(std::uint8_t) { return 1; }
	std::int32_t set_integration_time(double) { return 1; }
};

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Controller //////////////////////////////////////
////////////////////////////////////////////////////////////////

enum controller_id_e_t { E_CONTROLLER_MASTER, E_CONTROLLER_PARTNER };
enum controller_analog_e_t { E_CONTROLLER_ANALOG_LEFT_X, E_CONTROLLER_ANALOG_LEFT_Y, E_CONTROLLER_ANALOG_RIGHT_X, E_CONTROLLER_ANALOG_RIGHT_Y };
enum controller_digital_e_t {
	E_CONTROLLER_DIGITAL_L1 = 6, E_CONTROLLER_DIGITAL_L2, E_CONTROLLER_DIGITAL_R1, E_CONTROLLER_DIGITAL_R2,
	E_CONTROLLER_DIGITAL_UP, E_CONTROLLER_DIGITAL_DOWN, E_CONTROLLER_DIGITAL_LEFT, E_CONTROLLER_DIGITAL_RIGHT,
	E_CONTROLLER_DIGITAL_X, E_CONTROLLER_DIGITAL_B, E_CONTROLLER_DIGITAL_Y, E_CONTROLLER_DIGITAL_A
};

// No driver in the sim
struct Controller {
	Controller(controller_id_e_t) {}
	std::int32_t get_digital(controller_digital_e_t) { return 0; }
	std::int32_t get_digital_new_press(controller_digital_e_t) { return 0; }
	std::int32_t get_analog(controller_analog_e_t) { return 0; }
	template <class... Args> std::int32_t print(std::uint8_t, std::uint8_t, const char*, Args...) { return 1; }
	std::int32_t rumble(const char*) { return 1; }
};

#pragma endregion
////////////////////////////////////////////////////////////////

} // namespace pros
