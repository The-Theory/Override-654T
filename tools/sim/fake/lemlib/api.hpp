#pragma once

#include "pros/fake.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>

/**
 * Light stand-in for LemLib 0.5.6: same API and odom math, simplified motions that drive the
 * motors with the robot's own PID gains and exit conditions. Path shapes are close, not exact.
 * ponytail: build LemLib from source if motion shape ever matters more than auton logic
 */
namespace lemlib {

////////////////////////////////////////////////////////////////
#pragma region Math ////////////////////////////////////////////
////////////////////////////////////////////////////////////////

constexpr float radToDeg(float rad) { return rad * 180 / M_PI; }
constexpr float degToRad(float deg) { return deg * M_PI / 180; }
template <typename T> constexpr T sgn(T value) { return value < 0 ? -1 : 1; }

enum class AngularDirection { CW_CLOCKWISE, CCW_COUNTERCLOCKWISE, AUTO };

inline float angleError(float target, float position, bool radians = true, AngularDirection direction = AngularDirection::AUTO) {
	const float full = radians ? 2 * M_PI : 360;
	const float raw = std::fmod(std::fmod(target - position, full) + full, full);	// [0, full)
	if (direction == AngularDirection::CW_CLOCKWISE) return raw;
	if (direction == AngularDirection::CCW_COUNTERCLOCKWISE) return raw == 0 ? 0 : raw - full;
	return std::remainder(target - position, full);
}

class Pose {
public:
	float x, y, theta;
	Pose(float x, float y, float theta = 0) : x(x), y(y), theta(theta) {}
	Pose operator+(const Pose& o) const { return {x + o.x, y + o.y, theta}; }
	Pose operator-(const Pose& o) const { return {x - o.x, y - o.y, theta}; }
	float operator*(const Pose& o) const { return x * o.x + y * o.y; }
	Pose operator*(const float& k) const { return {x * k, y * k, theta}; }
	Pose operator/(const float& k) const { return {x / k, y / k, theta}; }
	Pose lerp(Pose o, float t) const { return {x + (o.x - x) * t, y + (o.y - y) * t, theta}; }
	float distance(Pose o) const { return std::hypot(x - o.x, y - o.y); }
	float angle(Pose o) const { return std::atan2(o.y - y, o.x - x); }
	Pose rotate(float a) const { return {x * std::cos(a) - y * std::sin(a), x * std::sin(a) + y * std::cos(a), theta}; }
};

// LemLib heading [deg] from one point toward another
inline float bearing(float fx, float fy, float tx, float ty) { return radToDeg(std::atan2(tx - fx, ty - fy)); }

class PID {
public:
	PID(float kP, float kI, float kD, float windupRange = 0, bool signFlipReset = false)
		: kP(kP), kI(kI), kD(kD), windupRange(windupRange), signFlipReset(signFlipReset) {}

	float update(float error) {
		integral += error;
		if (signFlipReset && sgn(error) != sgn(prevError)) integral = 0;
		if (windupRange != 0 && std::fabs(error) > windupRange) integral = 0;
		const float derivative = error - prevError;
		prevError = error;
		return error * kP + integral * kI + derivative * kD;
	}

	void reset() {
		integral  = 0;
		prevError = 0;
	}

protected:
	const float kP, kI, kD, windupRange;
	const bool signFlipReset;
	float integral = 0, prevError = 0;
};

// Done once the error has stayed inside range for time [ms]
class ExitCondition {
public:
	ExitCondition(float range, int time) : range(range), time(time) {}
	bool update(float error) {
		const std::uint32_t now = pros::millis();
		if (std::fabs(error) > range) start = -1;
		else if (start < 0) start = now;
		done = start >= 0 && now - start >= (std::uint32_t)time;
		return done;
	}
	bool getExit() const { return done; }
	void reset() { start = -1; done = false; }

private:
	float range;
	int time;
	std::int64_t start = -1;
	bool done = false;
};

class DriveCurve {
public:
	virtual float curve(float input) = 0;
};

class ExpoDriveCurve : public DriveCurve {
public:
	ExpoDriveCurve(float deadband, float minOutput, float curve) : deadband(deadband), minOutput(minOutput), curveGain(curve) {}
	float curve(float input) override {
		if (std::fabs(input) <= deadband) return 0;
		const float g = curveGain;
		const float out = (std::pow(g, std::fabs(input) - 127) * (std::fabs(input) - 127) + 127) / 127 * (127 - minOutput) + minOutput;
		return sgn(input) * out;
	}

private:
	const float deadband, minOutput, curveGain;
};

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Config //////////////////////////////////////////
////////////////////////////////////////////////////////////////

namespace Omniwheel {
constexpr float NEW_2 = 2.125;
constexpr float NEW_275 = 2.75;
constexpr float OLD_275 = 2.75;
constexpr float NEW_275_HALF = 2.744;
constexpr float OLD_275_HALF = 2.74;
constexpr float NEW_325 = 3.25;
constexpr float OLD_325 = 3.25;
constexpr float NEW_325_HALF = 3.246;
constexpr float OLD_325_HALF = 3.246;
constexpr float NEW_4 = 4;
constexpr float OLD_4 = 4.18;
constexpr float NEW_4_HALF = 3.995;
constexpr float OLD_4_HALF = 4.175;
}

class TrackingWheel {
public:
	TrackingWheel(pros::Rotation* encoder, float wheelDiameter, float distance, float gearRatio = 1)
		: encoder(encoder), diameter(wheelDiameter), distance(distance), gearRatio(gearRatio) {}
	float getDistanceTraveled() { return encoder->get_position() / 36000.0f * M_PI * diameter / gearRatio; }
	float getOffset() { return distance; }
	void reset() { encoder->reset_position(); }

	pros::Rotation* encoder;
	float diameter, distance, gearRatio;
};

struct OdomSensors {
	OdomSensors(TrackingWheel* vertical1, TrackingWheel* vertical2, TrackingWheel* horizontal1,
				TrackingWheel* horizontal2, pros::Imu* imu)
		: vertical1(vertical1), vertical2(vertical2), horizontal1(horizontal1), horizontal2(horizontal2), imu(imu) {}
	TrackingWheel *vertical1, *vertical2, *horizontal1, *horizontal2;
	pros::Imu* imu;
};

struct ControllerSettings {
	ControllerSettings(float kP, float kI, float kD, float windupRange, float smallError, float smallErrorTimeout,
					   float largeError, float largeErrorTimeout, float slew)
		: kP(kP), kI(kI), kD(kD), windupRange(windupRange), smallError(smallError), smallErrorTimeout(smallErrorTimeout),
		  largeError(largeError), largeErrorTimeout(largeErrorTimeout), slew(slew) {}
	float kP, kI, kD, windupRange, smallError, smallErrorTimeout, largeError, largeErrorTimeout, slew;
};

struct Drivetrain {
	Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, float trackWidth, float wheelDiameter,
			   float rpm, float horizontalDrift)
		: leftMotors(leftMotors), rightMotors(rightMotors), trackWidth(trackWidth), wheelDiameter(wheelDiameter),
		  rpm(rpm), horizontalDrift(horizontalDrift) {}
	pros::MotorGroup *leftMotors, *rightMotors;
	float trackWidth, wheelDiameter, rpm, horizontalDrift;
};

struct TurnToPointParams {
	bool forwards = true;
	AngularDirection direction = AngularDirection::AUTO;
	int maxSpeed = 127;
	int minSpeed = 0;
	float earlyExitRange = 0;
};

struct TurnToHeadingParams {
	AngularDirection direction = AngularDirection::AUTO;
	int maxSpeed = 127;
	int minSpeed = 0;
	float earlyExitRange = 0;
};

struct MoveToPoseParams {
	bool forwards = true;
	float horizontalDrift = 0;
	float lead = 0.6;
	float maxSpeed = 127;
	float minSpeed = 0;
	float earlyExitRange = 0;
};

struct MoveToPointParams {
	bool forwards = true;
	float maxSpeed = 127;
	float minSpeed = 0;
	float earlyExitRange = 0;
};

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Chassis /////////////////////////////////////////
////////////////////////////////////////////////////////////////

class Chassis {
public:
	Chassis(Drivetrain drivetrain, ControllerSettings lateralSettings, ControllerSettings angularSettings,
			OdomSensors sensors, DriveCurve* throttleCurve = nullptr, DriveCurve* steerCurve = nullptr)
		: lateralPID(lateralSettings.kP, lateralSettings.kI, lateralSettings.kD, lateralSettings.windupRange, true),
		  angularPID(angularSettings.kP, angularSettings.kI, angularSettings.kD, angularSettings.windupRange, true),
		  lateralSettings(lateralSettings), angularSettings(angularSettings), drivetrain(drivetrain), sensors(sensors) {
		sim::World& w = sim::world();
		w.left = drivetrain.leftMotors->get_port_all();
		w.right = drivetrain.rightMotors->get_port_all();
		w.trackWidth = drivetrain.trackWidth;
		w.wheelDiameter = drivetrain.wheelDiameter;
		w.wheelRpm = drivetrain.rpm;
		if (TrackingWheel* v = sensors.vertical1) {
			w.trackingPort = v->encoder->get_port();
			w.trackingDiameter = v->diameter;
			w.trackingOffset = v->distance;
		}
	}

	// Starts odom like LemLib does, so it needs PROS (the scheduler) up
	void calibrate(bool = true) {
		if (tracking) return;
		tracking = true;
		pros::Task([this] { odomLoop(); });
		pros::delay(10);
	}

	void setPose(float x, float y, float theta, bool radians = false) {
		if (radians) theta = radToDeg(theta);
		pose = {x, y, theta};
		sim::world().place(x, y, theta);
		publish();
	}
	void setPose(Pose p, bool radians = false) { setPose(p.x, p.y, p.theta, radians); }

	Pose getPose(bool radians = false, bool standardPos = false) {
		Pose p = pose;
		if (standardPos) p.theta = 90 - p.theta;
		if (radians) p.theta = degToRad(p.theta);
		return p;
	}

	void setBrakeMode(pros::motor_brake_mode_e mode) {
		drivetrain.leftMotors->set_brake_mode_all(mode);
		drivetrain.rightMotors->set_brake_mode_all(mode);
	}

	bool isInMotion() const { return running; }
	void cancelMotion() { running = false; }
	void cancelAllMotions() { running = false; }
	void waitUntilDone() { while (running) pros::delay(10); }
	void waitUntil(float dist) { while (running && traveled < dist) pros::delay(10); }

	void tank(int left, int right, bool = false) {
		drivetrain.leftMotors->move(left);
		drivetrain.rightMotors->move(right);
	}
	void arcade(int throttle, int turn, bool = false, float = 0.5) { tank(throttle + turn, throttle - turn); }
	void curvature(int throttle, int turn, bool = false) { arcade(throttle, turn); }

	void turnToHeading(float theta, int timeout, TurnToHeadingParams p = {}, bool async = true) {
		motion("turnToHeading", timeout, async, [=, this](Motion& m) {
			return turn(m, [theta](const Pose&) { return theta; }, p.direction, p.maxSpeed, p.minSpeed, p.earlyExitRange);
		});
	}

	void turnToPoint(float x, float y, int timeout, TurnToPointParams p = {}, bool async = true) {
		motion("turnToPoint", timeout, async, [=, this](Motion& m) {
			return turn(m, [=](const Pose& at) { return bearing(at.x, at.y, x, y) + (p.forwards ? 0 : 180); },
						p.direction, p.maxSpeed, p.minSpeed, p.earlyExitRange);
		});
	}

	void moveToPoint(float x, float y, int timeout, MoveToPointParams p = {}, bool async = true) {
		motion("moveToPoint", timeout, async, [=, this](Motion& m) {
			return drive(m, x, y, NAN, p.forwards, p.maxSpeed, p.minSpeed, p.earlyExitRange, 0, 0);
		});
	}

	void moveToPose(float x, float y, float theta, int timeout, MoveToPoseParams p = {}, bool async = true) {
		const float drift = p.horizontalDrift != 0 ? p.horizontalDrift : drivetrain.horizontalDrift;
		motion("moveToPose", timeout, async, [=, this](Motion& m) {
			return drive(m, x, y, theta, p.forwards, p.maxSpeed, p.minSpeed, p.earlyExitRange, p.lead, drift);
		});
	}

	PID lateralPID;
	PID angularPID;

private:
	ControllerSettings lateralSettings, angularSettings;
	Drivetrain drivetrain;
	OdomSensors sensors;
	Pose pose{0, 0, 0};
	bool tracking = false;
	bool running = false;
	float traveled = 0;

	struct Motion {
		Pose start;
		bool close = false;
		bool first = true;
		bool side = false;
		float maxSpeed = 127;
		float prevLateral = 0;
	};
	// One 10 ms update: writes left/right power, returns why it ended or nullptr to keep going
	using Tick = std::function<const char*(Motion&)>;

	void publish() {
		sim::World& w = sim::world();
		w.odomX = pose.x;
		w.odomY = pose.y;
		w.odomTheta = pose.theta;
	}

	// LemLib odom with one vertical wheel and an IMU: arc between updates, no sideways travel
	void odomLoop() {
		TrackingWheel* wheel = sensors.vertical1;
		float prevTravel = wheel ? wheel->getDistanceTraveled() : 0;
		float prevImu = sensors.imu->get_rotation();
		while (true) {
			const float travel = wheel ? wheel->getDistanceTraveled() : 0;
			const float imu = sensors.imu->get_rotation();
			const float dY = travel - prevTravel;
			const float dTheta = degToRad(imu - prevImu);
			prevTravel = travel;
			prevImu = imu;

			const float local = dTheta == 0 ? dY : 2 * std::sin(dTheta / 2) * (dY / dTheta + (wheel ? wheel->getOffset() : 0));
			const float mid = degToRad(pose.theta) + dTheta / 2;
			pose.x += local * std::sin(mid);
			pose.y += local * std::cos(mid);
			pose.theta += radToDeg(dTheta);
			traveled += std::fabs(local);
			publish();
			pros::delay(10);
		}
	}

	// LemLib queues a motion behind the running one, then runs it in a task if async
	void motion(const char* name, int timeout, bool async, Tick tick) {
		while (running) pros::delay(10);
		running = true;
		traveled = 0;
		auto body = [=, this] {
			lateralPID.reset();
			angularPID.reset();
			Motion m{pose};
			const std::uint32_t start = pros::millis();
			const char* why = "cancelled";
			ExitCondition latSmall(lateralSettings.smallError, lateralSettings.smallErrorTimeout);
			ExitCondition latLarge(lateralSettings.largeError, lateralSettings.largeErrorTimeout);
			ExitCondition angSmall(angularSettings.smallError, angularSettings.smallErrorTimeout);
			ExitCondition angLarge(angularSettings.largeError, angularSettings.largeErrorTimeout);
			exits = {&latSmall, &latLarge, &angSmall, &angLarge};
			while (running) {
				if (pros::millis() - start >= (std::uint32_t)timeout) { why = "TIMEOUT"; break; }
				if (const char* end = tick(m)) { why = end; break; }
				pros::delay(10);
			}
			tank(0, 0);
			sim::world().event("%s %s after %u ms at (%.1f, %.1f, %.0f)", name, why, pros::millis() - start, pose.x, pose.y, pose.theta);
			running = false;
		};
		if (!async) return body();
		pros::Task{body};
		pros::delay(10);
	}

	struct Exits { ExitCondition *latSmall, *latLarge, *angSmall, *angLarge; } exits{};

	static float clampAbs(float v, float max) { return std::clamp(v, -max, max); }
	static float atLeast(float v, float min) { return std::fabs(v) < min ? sgn(v) * min : v; }

	// Left/right power with turning kept, scaled so neither side passes max
	void apply(float lateral, float angular, float max) {
		float l = lateral + angular, r = lateral - angular;
		const float ratio = std::max(std::fabs(l), std::fabs(r)) / max;
		if (ratio > 1) { l /= ratio; r /= ratio; }
		tank(std::lround(l), std::lround(r));
	}

	const char* turn(Motion& m, std::function<float(const Pose&)> target, AngularDirection dir, float maxSpeed, float minSpeed, float earlyExit) {
		const float goal = target(pose);
		float error = angleError(goal, pose.theta, false, m.close ? AngularDirection::AUTO : dir);
		if (std::fabs(error) < 20) m.close = true;	// Past the forced direction's long way round
		if (exits.angSmall->update(error) || exits.angLarge->update(error)) return "settled";
		const bool side = error > 0;
		if (minSpeed > 0 && (std::fabs(error) < earlyExit || (!m.first && side != m.side))) return "exited early";
		m.side = side;
		m.first = false;
		float out = clampAbs(angularPID.update(error), maxSpeed);
		if (minSpeed > 0) out = atLeast(out, minSpeed);
		tank(std::lround(out), -std::lround(out));
		return nullptr;
	}

	// moveToPoint when theta is NaN, else boomerang moveToPose
	const char* drive(Motion& m, float x, float y, float theta, bool forwards, float maxSpeed, float minSpeed,
					  float earlyExit, float lead, float drift) {
		const bool pose2 = !std::isnan(theta);
		Pose at = pose;
		if (!forwards) at.theta += 180;
		const float targetTheta = pose2 ? theta + (forwards ? 0 : 180) : 0;
		const float dist = std::hypot(x - at.x, y - at.y);
		if (dist < 7.5 && !m.close) {
			m.close = true;
			m.maxSpeed = std::max(std::fabs(m.prevLateral), 60.0f);
		}
		if (!m.close) m.maxSpeed = maxSpeed;

		// Boomerang: chase a carrot behind the target along its heading
		float cx = x, cy = y;
		if (pose2 && !m.close) {
			cx -= std::sin(degToRad(targetTheta)) * lead * dist;
			cy -= std::cos(degToRad(targetTheta)) * lead * dist;
		}
		const float toCarrot = bearing(at.x, at.y, cx, cy);
		const float angular = m.close ? (pose2 ? angleError(targetTheta, at.theta, false) : 0) : angleError(toCarrot, at.theta, false);
		float lateralError = std::hypot(cx - at.x, cy - at.y) * std::cos(degToRad(angleError(at.theta, toCarrot, false)));
		if (m.close) lateralError = dist * std::cos(degToRad(angleError(at.theta, bearing(at.x, at.y, x, y), false)));

		const bool latDone = exits.latSmall->update(lateralError) || exits.latLarge->update(lateralError);
		if (pose2) {
			exits.angSmall->update(angular);
			exits.angLarge->update(angular);
		}
		if (latDone && (!pose2 || m.close)) return "settled";

		// Crossing the line through the target, square to the approach, ends a chained move
		const float ref = pose2 ? targetTheta : bearing(m.start.x, m.start.y, x, y);
		const bool side = (at.x - x) * std::sin(degToRad(ref)) + (at.y - y) * std::cos(degToRad(ref)) > -earlyExit;
		if (minSpeed > 0 && (m.close || !pose2) && side) return "exited early";

		float lateral = clampAbs(lateralPID.update(lateralError), m.maxSpeed);
		float turnOut = angularPID.update(angular);
		if (pose2 && !m.close && drift > 0) {
			// Slow down on tight curves, like LemLib's horizontal drift limit
			const float side2 = std::hypot(cx - at.x, cy - at.y);
			const float offset = std::fabs(std::sin(degToRad(angleError(toCarrot, at.theta, false)))) * side2;
			const float radius = offset < 1e-3 ? INFINITY : side2 * side2 / (2 * offset);
			lateral = clampAbs(lateral, std::sqrt(drift * radius * 9.8f));
		}
		if (!m.close) {
			lateral = std::max(lateral, 0.0f);
			if (minSpeed > 0) lateral = atLeast(lateral, minSpeed);
		}
		m.prevLateral = lateral;
		apply(forwards ? lateral : -lateral, turnOut, m.maxSpeed);
		return nullptr;
	}
};

#pragma endregion
////////////////////////////////////////////////////////////////

} // namespace lemlib
