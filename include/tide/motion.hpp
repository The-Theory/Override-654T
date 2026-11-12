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

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

// Planned moves: what the drive needs (Feedforward) and when it should be where (Trapezoid).
// No PROS calls, so it can be checked off-robot; Odom runs them on hardware

namespace tide {

////////////////////////////////////////////////////////////////
#pragma region Feedforward /////////////////////////////////////
////////////////////////////////////////////////////////////////

// Voltage a plant needs to hold a speed and accelerate, so feedback only fixes what's left
struct Feedforward {
	double kS = 0;	// [mV] to break friction
	double kV = 0;	// [mV per plant unit/s]
	double kA = 0;	// [mV per plant unit/s²]; grows with inertia, like a raised lift

	double mv(double v, double a) const { return ((v > 0) - (v < 0)) * kS + kV * v + kA * a; }
	bool   measured() const { return kV > 0 && kA > 0; }

	// Part way to other, t in 0..1; an unmeasured other leaves this as is
	Feedforward toward(const Feedforward& other, double t) const {
		if (!other.measured()) return *this;
		t = std::clamp(t, 0.0, 1.0);
		return {kS + (other.kS - kS) * t, kV + (other.kV - kV) * t, kA + (other.kA - kA) * t};
	}

	// Fastest it can go and speed up from standstill, using frac of the battery
	double topSpeed(double frac = 1) const { return (12000 * frac - kS) / kV; }
	double topAccel(double frac = 1) const { return (12000 * frac - kS) / kA; }
};

// Position and speed feedback that settles tracking error without overshoot in about 4 / bandwidth
// seconds. From the plant's own kV and kA, so a heavier robot gets stiffer gains on its own
struct Feedback {
	double kP;	// [mV per plant unit]
	double kD;	// [mV per plant unit/s]

	// Error dynamics kA·e'' + (kV + kD)·e' + kP·e = 0, critically damped at bandwidth [rad/s]
	static Feedback forPlant(const Feedforward& ff, double bandwidth) {
		return {ff.kA * bandwidth * bandwidth, std::max(0.0, 2 * ff.kA * bandwidth - ff.kV)};
	}
};

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Trapezoid ///////////////////////////////////////
////////////////////////////////////////////////////////////////

/**
 * Speed plan over a distance: speed up at maxAccel, cruise at maxSpeed, brake at maxAccel to
 * land on the end stopped. Short moves never reach maxSpeed, so they're a triangle.
 * Negative distance runs the same plan backwards.
 */
class Trapezoid {
public:
	struct State { double position, velocity, acceleration; };

	Trapezoid(double distance, double maxSpeed, double maxAccel)
		: sign(distance < 0 ? -1 : 1), length(std::fabs(distance)), accel(maxAccel) {
		cruise = std::min(maxSpeed, std::sqrt(length * accel));	// Triangle peak if it can't reach maxSpeed
		ramp   = cruise / accel;
		hold   = (length - cruise * ramp) / cruise;
		if (length == 0) cruise = ramp = hold = 0;
	}

	double duration() const { return 2 * ramp + hold; }

	State at(double t) const {
		State s;
		if (t <= 0) s = {0, 0, 0};
		else if (t < ramp) s = {accel * t * t / 2, accel * t, accel};
		else if (t < ramp + hold) s = {cruise * ramp / 2 + cruise * (t - ramp), cruise, 0};
		else if (t < duration()) {
			const double left = duration() - t;
			s = {length - accel * left * left / 2, accel * left, -accel};
		}
		else s = {length, 0, 0};
		return {sign * s.position, sign * s.velocity, sign * s.acceleration};
	}

private:
	double sign, length, accel;
	double cruise = 0, ramp = 0, hold = 0;	// [unit/s], [s], [s]
};

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Ramsete /////////////////////////////////////////
////////////////////////////////////////////////////////////////

// Where the robot should be at t along a path, and how fast. Headings clockwise from +y like LemLib
struct Waypoint {
	double t;			// [s]
	double x, y, theta;	// [in], [deg]
	double v = 0, w = 0;	// [in/s] forward, [deg/s] clockwise
	double a = 0, alpha = 0;	// [in/s²], [deg/s²]
};

/**
 * Speed and turn rate {in/s, deg/s} that pull a robot at (x, y, theta) back onto ref while it
 * keeps moving (RAMSETE). Corrects sideways error by steering, since a tank can't strafe.
 * - b: how hard sideways error steers [1/in²]; zeta: damping, 0.7 settles without overshoot
 */
inline std::pair<double, double> ramsete(const Waypoint& ref, double x, double y, double theta,
										 double b = 0.0013, double zeta = 0.7) {
	constexpr double MIN_GAIN = 2;	// [1/s] RAMSETE's own gain is zero while the path stands still
	const double rad = std::numbers::pi / 180;
	const double h = theta * rad;
	const double dx = ref.x - x, dy = ref.y - y;
	const double ahead = dx * std::sin(h) + dy * std::cos(h);
	const double right = dx * std::cos(h) - dy * std::sin(h);
	const double turn  = std::remainder(ref.theta - theta, 360.0) * rad;	// clockwise to ref
	const double wr = ref.w * rad;

	const double k = std::max(MIN_GAIN, 2 * zeta * std::sqrt(wr * wr + b * ref.v * ref.v));
	const double sinc = std::fabs(turn) < 1e-6 ? 1 : std::sin(turn) / turn;
	const double v = ref.v * std::cos(turn) + k * ahead;
	const double w = wr + k * turn + b * ref.v * sinc * right;
	return {v, w / rad};
}

#pragma endregion
////////////////////////////////////////////////////////////////

} // namespace tide
