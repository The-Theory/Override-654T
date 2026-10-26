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

#include "tide/mechanism.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <numbers>



namespace tide {

////////////////////////////////////////////////////////////////
#pragma region Settings ////////////////////////////////////////
////////////////////////////////////////////////////////////////

// Anything with a position to hold: how to read it, and how to push it
struct Plant {
	std::function<double()> measure;		// in the units the gains will use
	std::function<void(double)> apply;		// [mV], + must move measure() up
};

// Ziegler-Nichols variants, most to least aggressive
enum class TuneRule { CLASSIC, SOME_OVERSHOOT, NO_OVERSHOOT };

struct TuneSettings {
	double setpoint;				// oscillates around this [plant units]
	double amplitude;				// relay swing [mV]; must beat static friction and the voltage needed to hold the load
	double maxDeviation;			// aborts if it strays this far once oscillating [plant units]
	double hysteresis    = 0.5;		// noise band so the relay doesn't chatter [plant units]
	int    warmupCycles  = 2;		// lets the holding voltage settle before measuring
	int    measureCycles = 4;
	int    timeout       = 15000;	// [ms]
	TuneRule rule        = TuneRule::NO_OVERSHOOT;
};

struct TuneResult {
	bool   ok = false;
	double kP = 0, kI = 0, kD = 0;	// per 10 ms update, same form as Gains and lemlib::PID [mV/unit]
	double kG = 0;					// holding voltage at the setpoint, Gains::kG [mV]
	double ku = 0;					// ultimate gain [mV/unit]
	double tu = 0;					// oscillation period [s]
};

// lemlib::PID outputs -127..127, not mV
constexpr double LEMLIB_SCALE = 127.0 / 12000.0;

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region RelayTuner //////////////////////////////////////
////////////////////////////////////////////////////////////////

/**
 * Relay (Åström-Hägglund) auto-tune: bang-bangs around the setpoint, measures the
 * oscillation it settles into, and turns that into PID gains.
 * - The middle of the swing drifts to whatever holds the load (gravity), which becomes kG.
 * - No PROS calls, so it can be checked off-robot; autotune() runs it on hardware.
 */
class RelayTuner {
public:
	static constexpr double DT = 0.01;	// [s] update period of Mechanism and lemlib::PID, which per-update gains assume

	explicit RelayTuner(TuneSettings settings) : s(settings) {}

	// Feeds one reading taken at t [s]; returns the voltage to apply
	double step(double y, double t) {
		const double error = s.setpoint - y;
		maxY = std::max(maxY, y);
		minY = std::min(minY, y);

		if (crossed && std::fabs(error) > s.maxDeviation) aborted = true;

		if (!high && error > s.hysteresis) {
			high = true;
			cycleDone(t, y);
		} else if (high && error < -s.hysteresis) {
			high = false;
			crossed = true;
			lowStart = t;
		}
		return std::clamp(bias + (high ? s.amplitude : -s.amplitude), -12000.0, 12000.0);
	}

	bool failed() const { return aborted; }
	bool done() const { return cycles >= s.warmupCycles + s.measureCycles; }

	TuneResult result() const {
		TuneResult r;
		const int n = cycles - s.warmupCycles;
		if (aborted || n <= 0) return r;

		const double a = ampSum / n;
		if (a <= s.hysteresis) return r;
		r.tu = periodSum / n;
		r.kG = biasSum / n;
		r.ku = 4 * s.amplitude / (std::numbers::pi * std::sqrt(a * a - s.hysteresis * s.hysteresis));

		// Kp, Ti, Td as fractions of Ku and Tu
		double kp = 0.2, ti = 0.5, td = 1.0 / 3;
		if (s.rule == TuneRule::CLASSIC)        { kp = 0.6;  td = 1.0 / 8; }
		if (s.rule == TuneRule::SOME_OVERSHOOT) { kp = 0.33; }
		kp *= r.ku;
		ti *= r.tu;
		td *= r.tu;

		r.kP = kp;
		r.kI = kp / ti * DT;
		r.kD = kp * td / DT;
		r.ok = true;
		return r;
	}

private:
	const TuneSettings s;

	bool   high      = false;
	bool   crossed   = false;	// Reached the setpoint once, so the deviation guard applies
	bool   aborted   = false;
	double bias      = 0;
	double highStart = -1;		// -1 until the first full high phase starts
	double lowStart  = 0;
	double maxY      = -INFINITY;
	double minY      =  INFINITY;

	int    cycles    = 0;
	double ampSum    = 0;
	double periodSum = 0;
	double biasSum   = 0;

	// Low -> high switch closes one oscillation
	void cycleDone(double t, double y) {
		if (highStart >= 0 && crossed) {
			const double tHigh = lowStart - highStart;
			const double tLow  = t - lowStart;
			if (++cycles > s.warmupCycles) {
				ampSum    += (maxY - minY) / 2;
				periodSum += t - highStart;
				biasSum   += bias;
			}
			// Gravity makes one half-swing slower; shift the middle until both take equally long
			bias = std::clamp(bias + 0.5 * s.amplitude * (tHigh - tLow) / (tHigh + tLow),
				-12000.0 + s.amplitude, 12000.0 - s.amplitude);
		}
		highStart = t;
		maxY = minY = y;
	}
};

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Plants //////////////////////////////////////////
////////////////////////////////////////////////////////////////

// Mechanism position in its own units; tuned gains drop straight into its Gains
inline Plant mechanismPlant(Mechanism& mech) {
	return {[&mech] { return mech.position(); }, [&mech](double mv) { mech.drive(std::lround(mv)); }};
}

// Heading [deg] from where the tune starts; gains go in lemlib angular ControllerSettings (× LEMLIB_SCALE)
inline Plant turnPlant(pros::MotorGroup& left, pros::MotorGroup& right, pros::Imu& imu) {
	const double start = imu.get_rotation();
	return {[&imu, start] { return imu.get_rotation() - start; },
			[&left, &right](double mv) { left.move_voltage(mv); right.move_voltage(-mv); }};
}

// Tracking wheel distance [in] from where the tune starts; gains go in lemlib lateral ControllerSettings (× LEMLIB_SCALE)
inline Plant drivePlant(pros::MotorGroup& left, pros::MotorGroup& right, pros::Rotation& tracking, double wheelDiameter) {
	const double start = tracking.get_position();
	return {[&tracking, start, wheelDiameter] { return (tracking.get_position() - start) / 36000.0 * std::numbers::pi * wheelDiameter; },
			[&left, &right](double mv) { left.move_voltage(mv); right.move_voltage(mv); }};
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Autotune ////////////////////////////////////////
////////////////////////////////////////////////////////////////

/**
 * Relay-tunes a plant, blocking until done, then cuts its power and prints gains
 * to the brain screen (lines 2-6) and the terminal.
 * - Give it room: it swings back and forth around the setpoint for several seconds.
 */
inline TuneResult autotune(const char* name, const Plant& plant, TuneSettings settings) {
	RelayTuner tuner(settings);
	const std::uint32_t start = pros::millis();
	std::uint32_t wake = start;
	while (!tuner.done() && !tuner.failed() && pros::millis() - start < (std::uint32_t)settings.timeout) {
		plant.apply(tuner.step(plant.measure(), (pros::millis() - start) / 1000.0));
		pros::Task::delay_until(&wake, 10);	// Steady 10 ms, or the measured period skews
	}
	plant.apply(0);

	const TuneResult r = tuner.result();
	char lines[5][64];
	std::snprintf(lines[0], 64, "Tune %s: %s", name, r.ok ? "OK" : (tuner.failed() ? "FAILED (strayed)" : "FAILED (no oscillation, raise amplitude)"));
	std::snprintf(lines[1], 64, "kP %.2f  kI %.4f  kD %.1f", r.kP, r.kI, r.kD);
	std::snprintf(lines[2], 64, "kG %.0f mV", r.kG);
	std::snprintf(lines[3], 64, "LemLib kP %.3f kI %.5f kD %.2f", r.kP * LEMLIB_SCALE, r.kI * LEMLIB_SCALE, r.kD * LEMLIB_SCALE);
	std::snprintf(lines[4], 64, "Ku %.1f  Tu %.3f s", r.ku, r.tu);
	for (int i = 0; i < 5; i++) {
		pros::lcd::print(i + 2, "%s", lines[i]);
		std::printf("%s\n", lines[i]);
	}
	return r;
}

#pragma endregion
////////////////////////////////////////////////////////////////

} // namespace tide
