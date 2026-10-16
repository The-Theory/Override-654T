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
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>



namespace tide {

////////////////////////////////////////////////////////////////
#pragma region Mechanism ///////////////////////////////////////
////////////////////////////////////////////////////////////////
// PID tunings for a Mechanism
struct Gains {
	double kP;          // [mV/deg]
	double kI;
	double kD;          // Per update, not per second
	double windup = 0;  // Integral only builds inside this error, 0 means always [deg]
	double settle = 0;  // Inside this error the motor brakes instead of correcting [deg]
};

/**
 * Motor held at a target by PID on an external rotation sensor.
 * Moves are non-blocking, like move_absolute().
 * - Call start() in initialize(), since tasks can't be made before PROS is up.
 */
class Mechanism {
public:
	/**
	 * ratio scales sensor degrees into the units targets are given in,
	 * e.g. motor degrees so old move_absolute() values still work.
	 */
	Mechanism(pros::AbstractMotor& motor, pros::Rotation& sensor, Gains gains, double ratio = 1.0)
		: motor(motor), sensor(sensor), gains(gains), ratio(ratio) {}

	// Zeroes the sensor and starts holding there
	void start() {
		tare();
		moveTo(0);
		pros::Task([this] { loop(); });
	}

	void tare() {
		sensor.reset_position();
		target = 0;
	}

	double position() const { return sensor.get_position() / 100.0 * ratio; }

	void moveTo(double to) {
		motor.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
		pidActive = true;  // Before target, or the loop could overwrite it with position()
		manualMv  = 0;
		target    = to;
	}

	// Relative to the target, so repeated moves don't stack drift
	void moveBy(double delta) { moveTo(target + delta); }

	// Holds wherever it is now
	void hold() { moveTo(position()); }

	// Overrides PID with a raw voltage until the next move
	void drive(int mv) {
		manualMv  = mv;
		pidActive = false;
	}

	// Goes limp until the next move
	void release() {
		motor.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
		drive(0);
	}

private:
	static constexpr int MAX_MV = 12000;

	pros::AbstractMotor& motor;
	pros::Rotation& sensor;
	const Gains gains;
	const double ratio;

	double target    = 0;
	int    manualMv  = 0;
	bool   pidActive = false;
	double integral  = 0;
	double prevError = 0;

	void resetPID() {
		integral  = 0;
		prevError = 0;
	}

	// Same math as lemlib::PID so gains carry over
	double pid(double error) {
		integral += error;
		if (std::signbit(error) != std::signbit(prevError)) integral = 0;
		if (gains.windup != 0 && std::fabs(error) > gains.windup) integral = 0;
		const double derivative = error - prevError;
		prevError = error;
		return error * gains.kP + integral * gains.kI + derivative * gains.kD;
	}

	void loop() {
		while (true) {
			// Unplugged or not-yet-ready sensor reads PROS_ERR, which would look like a huge error and drive at full power
			const std::int32_t raw = sensor.get_position();
			if (raw == PROS_ERR) {
				resetPID();
				motor.brake();
				pros::delay(10);
				continue;
			}
			const double pos   = raw / 100.0 * ratio;
			const double error = target - pos;
			if (!pidActive) {
				target = pos;  // So moveBy() after manual/release is relative to where it ended up
				resetPID();
				if (manualMv == 0) motor.brake();
				else motor.move_voltage(manualMv);
			} else if (std::fabs(error) < gains.settle) {
				resetPID();
				motor.brake();
			} else {
				motor.move_voltage(std::clamp(pid(error), -(double)MAX_MV, (double)MAX_MV));
			}
			pros::delay(10);
		}
	}
};
#pragma endregion
////////////////////////////////////////////////////////////////

}
