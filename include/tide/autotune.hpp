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
#include "tide/motion.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <numbers>
#include <string>
#include <utility>
#include <vector>



namespace tide {

////////////////////////////////////////////////////////////////
#pragma region Settings ////////////////////////////////////////
////////////////////////////////////////////////////////////////

// Anything with a position to hold: how to read it, and how to push it
struct Plant {
	std::function<double()> measure;		// in the units the gains will use
	std::function<void(double)> apply;		// [mV], + must move measure() up
	bool lemlib = false;					// gains go in lemlib ControllerSettings, not Gains
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
	std::string summary;			// what autotune() printed
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

// Heading [deg] from where the tune starts; gains go in lemlib angular ControllerSettings
inline Plant angularPlant(pros::MotorGroup& left, pros::MotorGroup& right, pros::Imu& imu) {
	const double start = imu.get_rotation();
	return {[&imu, start] { return imu.get_rotation() - start; },
			[&left, &right](double mv) { left.move_voltage(mv); right.move_voltage(-mv); }, true};
}

// Tracking wheel distance [in] from where the tune starts; gains go in lemlib lateral ControllerSettings
inline Plant lateralPlant(pros::MotorGroup& left, pros::MotorGroup& right, pros::Rotation& tracking, double wheelDiameter) {
	const double start = tracking.get_position();
	return {[&tracking, start, wheelDiameter] { return (tracking.get_position() - start) / 36000.0 * std::numbers::pi * wheelDiameter; },
			[&left, &right](double mv) { left.move_voltage(mv); right.move_voltage(mv); }, true};
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Autotune ////////////////////////////////////////
////////////////////////////////////////////////////////////////

/**
 * Relay-tunes a plant, blocking until done, then cuts its power and prints gains
 * to the terminal. The same text comes back in TuneResult::summary.
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

	TuneResult r = tuner.result();
	// Only what gets pasted into code
	char line[64];
	std::snprintf(line, 64, "Tune %s: %s\n", name, r.ok ? "OK" : (tuner.failed() ? "FAILED (strayed)" : "FAILED (no oscillation, raise amplitude)"));
	r.summary = line;
	if (r.ok && plant.lemlib) {
		std::snprintf(line, 64, "kP %.3f  kI %.5f  kD %.2f\n", r.kP * LEMLIB_SCALE, r.kI * LEMLIB_SCALE, r.kD * LEMLIB_SCALE);
		r.summary += line;
	} else if (r.ok) {
		std::snprintf(line, 64, "kP %.2f  kI %.4f  kD %.1f\nkG %.0f\n", r.kP, r.kI, r.kD, r.kG);
		r.summary += line;
	}
	std::printf("%s", r.summary.c_str());
	return r;
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Characterize ////////////////////////////////////
////////////////////////////////////////////////////////////////

// One reading during characterize(): what was applied and where the plant was
struct FeedforwardSample { double t, mv, position; };	// [s], [mV], [plant units]

/**
 * Least-squares fit of mv = kS·sign(v) + kV·v + kA·a over a run, with speed and
 * acceleration from central differences. No PROS calls, so it can be checked off-robot.
 * - Skips samples near standstill (friction there isn't kS), and coasting or braking ones,
 *   where the motor isn't what drives the motion
 */
inline Feedforward fitFeedforward(const std::vector<FeedforwardSample>& s, double minSpeed, int window = 5) {
	double m[3][4] = {};	// normal equations, augmented with the right side
	for (std::size_t i = window; i + window < s.size(); i++) {
		const double dt = s[i + window].t - s[i].t;
		const double v  = (s[i + window].position - s[i - window].position) / (2 * dt);
		const double a  = (s[i + window].position - 2 * s[i].position + s[i - window].position) / (dt * dt);
		if (std::fabs(v) < minSpeed || v * s[i].mv <= 0) continue;
		if (s[i - window].mv * s[i + window].mv <= 0) continue;	// Window spans a rest or the switch to another run
		const double row[3] = {double((v > 0) - (v < 0)), v, a};
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) m[r][c] += row[r] * row[c];
			m[r][3] += row[r] * s[i].mv;
		}
	}
	// Gaussian elimination with partial pivoting
	for (int c = 0; c < 3; c++) {
		int pivot = c;
		for (int r = c + 1; r < 3; r++) if (std::fabs(m[r][c]) > std::fabs(m[pivot][c])) pivot = r;
		std::swap(m[c], m[pivot]);
		if (std::fabs(m[c][c]) < 1e-9) return {NAN, NAN, NAN};	// Never moved, or never sped up
		for (int r = 0; r < 3; r++) {
			if (r == c) continue;
			const double f = m[r][c] / m[c][c];
			for (int k = c; k < 4; k++) m[r][k] -= f * m[c][k];
		}
	}
	return {m[0][3] / m[0][0], m[1][3] / m[1][1], m[2][3] / m[2][2]};
}

/**
 * Measures a plant's Feedforward from four runs, each way: slow voltage ramps show friction
 * and speed, hard steps show inertia. Prints the gains in plant units and returns the same text.
 * - Run it in each pose that changes the robot (lift down, lift up): kA moves the most
 * - Each pair comes back about where it started; drive needs ~2 ft clear front and back
 */
inline std::string characterize(const char* name, const Plant& plant, int rampMs = 4000, int stepMv = 5000, int stepMs = 1000) {
	constexpr double RAMP_RATE = 1000;	// [mV/s] slow, so most of the ramp is near steady speed
	constexpr int    REST_MS   = 400;	// stopped between runs, so each starts from standstill
	struct Run { bool ramp; int sign; };
	constexpr Run RUNS[] = {{true, 1}, {true, -1}, {false, 1}, {false, -1}};

	std::vector<FeedforwardSample> samples;
	const std::uint32_t start = pros::millis();
	std::uint32_t wake = start;
	double topSpeed = 0;	// [plant units/s] end of the first ramp, for a standstill cutoff that fits any units
	for (const Run& run : RUNS) {
		const int length = run.ramp ? rampMs : stepMs;
		const std::uint32_t runStart = pros::millis();
		for (std::uint32_t t = 0; t < (std::uint32_t)(length + REST_MS); t = pros::millis() - runStart) {
			const double mv = t >= (std::uint32_t)length ? 0 : run.sign * (run.ramp ? RAMP_RATE * t / 1000 : stepMv);
			samples.push_back({(pros::millis() - start) / 1000.0, mv, plant.measure()});
			plant.apply(mv);
			pros::Task::delay_until(&wake, 10);	// Steady 10 ms, or the differences skew
		}
		if (topSpeed == 0) {
			const std::size_t end = samples.size() - REST_MS / 10;
			topSpeed = std::fabs(samples[end].position - samples[end - 10].position) / 0.1;
		}
	}
	plant.apply(0);

	const Feedforward ff = fitFeedforward(samples, 0.05 * topSpeed);
	char text[96];
	std::snprintf(text, sizeof(text), "FF %s: kS %.0f  kV %.2f  kA %.2f\n", name, ff.kS, ff.kV, ff.kA);
	std::printf("%s", text);
	return text;
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Offset //////////////////////////////////////////
////////////////////////////////////////////////////////////////

/**
 * Spins in place and measures a vertical tracking wheel's offset [in] for lemlib::TrackingWheel,
 * + right of the turning center. Spinning in place should move the wheel by offset × angle,
 * so it needs no measuring, and drift averages out over several turns.
 */
inline std::string trackingOffset(pros::MotorGroup& left, pros::MotorGroup& right, pros::Imu& imu,
								  pros::Rotation& tracking, double wheelDiameter, int turns = 5, int mv = 4000) {
	const Plant spin  = angularPlant(left, right, imu);
	const Plant wheel = lateralPlant(left, right, tracking, wheelDiameter);
	spin.apply(mv);
	while (std::abs(spin.measure()) < turns * 360.0) pros::delay(10);
	spin.apply(0);
	pros::delay(500);	// Let it coast to a stop, so both readings cover the same motion

	// Clockwise spin moves a wheel right of center backward
	const double offset = -wheel.measure() / (spin.measure() * std::numbers::pi / 180);
	char line[96];
	// Raw travel too, so a wheel that never turned (no contact, unplugged) doesn't pass as a centered one
	std::snprintf(line, 96, "Tracking offset %.2f in\nwheel %.2f in over %.0f deg\n", offset, wheel.measure(), spin.measure());
	std::printf("%s", line);
	return line;
}

#pragma endregion
////////////////////////////////////////////////////////////////

} // namespace tide
