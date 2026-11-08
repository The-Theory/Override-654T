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

#include "lemlib/api.hpp"	// IWYU pragma: keep
#include "tide/field.hpp"
#include "tide/motion.hpp"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <numbers>
#include <optional>
#include <string>
#include <vector>



namespace tide {

// Where a distance sensor sits, measured from the tracking center
struct DistanceMount {
	pros::Distance& sensor;
	double right;	// [in], + is robot right
	double forward;	// [in], + is robot front
	double facing;	// [deg] clockwise from robot front
};

// Outside Odom: a nested struct's defaults can't be used as Odom's default argument
struct OdomSettings {
	double halfField      = 70.2;	// center to inside face of perimeter [in] (70.25 on metal perimeters)
	double maxResetRange  = 40;		// [in] sensor error is ~5% of range past 8", so far walls hurt more than help
	double maxResetAngle  = 15;		// [deg] off-perpendicular before beam can hit wall junk instead of the wall
	double maxResetJump   = 6;		// [in] a reset that moves the pose further than this read a robot or piece, not the wall
	int    stallGrace     = 200;	// [ms] to get moving before stall checks start
	int    stallVelocity  = 11000;	// [centideg/s] tracking wheel speed counted as stopped
	double toolRight      = 0;		// [in] scoring point, from the tracking center
	double toolForward    = 0;		// [in] negative for a tool on the back

	// Planned moves (profiledDrive/Turn), from the Tune page's FF runs; unmeasured falls back to LemLib's PID moves
	Feedforward lateral, lateralLoaded;		// [mV per in] unloaded, and at load() = 1 (lift up)
	Feedforward angular, angularLoaded;		// [mV per deg]
	std::function<double()> load;			// 0..1 toward the loaded gains, e.g. lift height; empty = always unloaded
	double bandwidth      = 6;		// [rad/s] how hard feedback chases the plan; higher is stiffer, too high buzzes
};

/**
 * Position helpers on top of a LemLib chassis that don't depend on battery or tiles:
 * distance-sensor wall resets, moves relative to the robot, moves that land the tool
 * (not the tracking center) on a target, and pushes into hard stops.
 */
class Odom {
public:
	Odom(lemlib::Chassis& chassis, pros::MotorGroup& left, pros::MotorGroup& right,
		 pros::Rotation& tracking, std::vector<DistanceMount> mounts, OdomSettings settings = {})
		: chassis(chassis), left(left), right(right), tracking(tracking), mounts(mounts), settings(settings) {}

	/**
	 * Corrects the odom axis normal to a wall from a distance reading, once the robot stops.
	 * The beam must be on bare perimeter (not a Loader, Toggle or wall Scoring Objects).
	 * Uses whichever sensor faces the wall; leaves the pose alone and prints why if none can.
	 */
	bool wallReset(Wall wall) {
		waitUntilStopped();

		const bool xAxis = wall == Wall::RED || wall == Wall::BLUE;
		const double sign = (wall == Wall::BLUE || wall == Wall::FAR) ? 1 : -1;
		const double wallHeading = xAxis ? (sign > 0 ? dir::EAST : dir::WEST) : (sign > 0 ? dir::NORTH : dir::SOUTH);
		const lemlib::Pose pose = chassis.getPose();
		const char* why = "no sensor faces it";

		for (const DistanceMount& mount : mounts) {
			const double beam = pose.theta + mount.facing;
			if (std::fabs(lemlib::angleError(beam, wallHeading, false)) > settings.maxResetAngle) continue;

			const double dist = medianInches(mount.sensor);
			if (std::isnan(dist) || dist > settings.maxResetRange) { why = "wall out of range"; continue; }

			const Point sensorAt = toField(mount.right, mount.forward, pose.theta);
			const Point beamHit  = toField(0, dist, beam);
			const double wallPos = sign * settings.halfField;
			const double fixed   = wallPos - (xAxis ? sensorAt.x + beamHit.x : sensorAt.y + beamHit.y);
			const double jump    = fixed - (xAxis ? pose.x : pose.y);
			// A beam on a robot or piece reads short; trusting it would throw the pose off
			if (std::fabs(jump) > settings.maxResetJump) { why = "beam hit something else"; continue; }

			if (xAxis) chassis.setPose(fixed, pose.y, pose.theta);
			else       chassis.setPose(pose.x, fixed, pose.theta);
			std::printf("Reset %s wall: %+.1f in\n", wallName(wall), jump);
			return true;
		}
		std::printf("Reset %s wall FAILED: %s\n", wallName(wall), why);
		return false;
	}

	/**
	 * Lands the tool `standoff` inches short of target, facing heading, and waits for it to settle.
	 * Coordinates get the robot close; finish onto anything whose real spot varies with creep().
	 * - Drives forwards when the trip runs along heading, else backs in, so it never loops around
	 */
	void approach(Point target, double heading, double standoff, int maxSpeed = 127, int timeout = 2000) {
		const Point at = centerFor(shortOf(target, heading, standoff), heading);
		const lemlib::Pose pose = chassis.getPose();
		if (profiled()) {
			const double dist = std::hypot(at.x - pose.x, at.y - pose.y);
			if (dist > 0.5) {
				// Forwards or backwards, whichever turns less in all
				const double face = bearing({pose.x, pose.y}, at);
				auto turning = [&](double facing) {
					return std::fabs(lemlib::angleError(facing, pose.theta, false)) + std::fabs(lemlib::angleError(heading, facing, false));
				};
				const bool forwards = turning(face) <= turning(face + 180);
				profiledTurn(forwards ? face : face + 180);
				const lemlib::Pose turned = chassis.getPose();	// Distance from here: it may have still been rolling
				const double left = std::hypot(at.x - turned.x, at.y - turned.y);
				profiledDrive(forwards ? left : -left, at, maxSpeed / 127.0);
			}
			if (std::fabs(lemlib::angleError(heading, chassis.getPose().theta, false)) > 1) profiledTurn(heading);
			return;
		}
		if (std::hypot(at.x - pose.x, at.y - pose.y) > 1) {
			const bool forwards = std::fabs(lemlib::angleError(bearing({pose.x, pose.y}, at), heading, false)) <= 90;
			// Nearly in line, steer onto the point and turn after: a tank can't fix a sideways
			// miss on a straight run, and the final turn pivots the tool onto target (centerFor)
			const bool inLine = std::fabs(lemlib::angleError(bearing({pose.x, pose.y}, at) + (forwards ? 0 : 180), heading, false)) < 15;
			if (inLine) chassis.moveToPoint(at.x, at.y, timeout, {.forwards = forwards, .maxSpeed = (float)maxSpeed}, false);
			else        chassis.moveToPose(at.x, at.y, heading, timeout, {.forwards = forwards, .maxSpeed = (float)maxSpeed}, false);
			waitUntilStopped();	// LemLib settles while still rolling; a turn now would carry it inches past
		}
		// moveToPose settles on position only; a big swing can end off heading
		if (std::fabs(lemlib::angleError(heading, chassis.getPose().theta, false)) > 2) chassis.turnToHeading(heading, 1000, {}, false);
	}

	// Same, facing straight at target from where the robot is
	void approachStraight(Point target, double standoff, int maxSpeed = 127, int timeout = 2000) {
		const lemlib::Pose pose = chassis.getPose();
		const double heading = bearing({pose.x, pose.y}, target) + (settings.toolForward >= 0 ? 0 : 180);
		approach(target, heading, standoff, maxSpeed, timeout);
	}

	/**
	 * Drives straight at mv until done() or timeout; true if done().
	 * For the last inches onto something a sensor can see, where coordinates aren't good enough.
	 * - [] { return false; } just pushes for timeout ms
	 */
	bool creep(int mv, const std::function<bool()>& done, int timeout = 1500) {
		stopMotions();
		drive(mv, mv);
		const std::uint32_t start = pros::millis();
		bool hit = false;
		while (!(hit = done()) && pros::millis() - start < (std::uint32_t)timeout) pros::delay(10);
		drive(0, 0);
		return hit;
	}

	/**
	 * Drives distance [in] (negative backs up) on a speed plan, holding the heading it starts on,
	 * or steering at aim until close. Feedforward does the work, Feedback::forPlant fixes the rest,
	 * so it lands stopped on the end without tuning. Needs settings.lateral and .angular measured.
	 * - speed: fraction of planned top speed
	 * - timeout [ms]: past the plan's own length before giving up
	 */
	void profiledDrive(double distance, std::optional<Point> aim = {}, double speed = 1, int timeout = 750) {
		stopMotions();
		const Trapezoid plan(distance, lateralNow().topSpeed(PLAN_BATTERY) * speed, lateralNow().topAccel(ACCEL_BATTERY));
		const lemlib::Pose start = chassis.getPose();
		const Point along = toField(0, 1, start.theta);
		double s = 0, v = 0, theta = start.theta, w = 0;	// [in], [in/s], [deg], [deg/s]

		const std::uint32_t t0 = pros::millis();
		std::uint32_t wake = t0;
		while (true) {
			const double t = (pros::millis() - t0) / 1000.0;
			const lemlib::Pose pose = chassis.getPose();
			const double now = (pose.x - start.x) * along.x + (pose.y - start.y) * along.y;
			v += ((now - s) / DT - v) * SMOOTHING;
			w += (std::remainder(pose.theta - theta, 360.0) / DT - w) * SMOOTHING;
			s = now;
			theta = pose.theta;

			const Trapezoid::State ref = plan.at(t);
			const double e = ref.position - s;
			if (t > plan.duration() && std::fabs(e) < SETTLED_IN && std::fabs(v) < SETTLED_IN_S) break;
			if (t > plan.duration() + timeout / 1000.0) {
				std::printf("profiledDrive TIMEOUT %.1f in short\n", e);
				break;
			}
			const double push = follow(lateralNow(), ref, e, v, SETTLED_IN);

			// Toward aim while the bearing is steady; the last inches just hold it
			double want = start.theta;
			if (aim && std::fabs(distance - s) > AIM_LOCK) want = bearing({pose.x, pose.y}, *aim) + (distance < 0 ? 180 : 0);
			const Feedback steer = Feedback::forPlant(angularNow(), settings.bandwidth);
			const double turn = steer.kP * lemlib::angleError(want, pose.theta, false) - steer.kD * w;
			drive(clampMv(push + turn), clampMv(push - turn));
			pros::Task::delay_until(&wake, 10);
		}
		drive(0, 0);
	}

	/**
	 * Turns slowly through the current heading ± sweep [deg] reading a distance sensor, and returns
	 * where each object it passed sits on the field [in]. For lining up on a piece that's off its
	 * nominal spot, or has neighbors to tell it from.
	 * - An object is a run of steady readings; a jump closer or further starts the next
	 * - radius: for round pieces, reports centers instead of the nearest point the beam touched
	 */
	std::vector<Point> scan(const DistanceMount& mount, double sweep, double radius = 0, double maxRange = 24) {
		const lemlib::Pose start = chassis.getPose();
		turnTo(start.theta - sweep);
		stopMotions();

		struct Reading { double theta, dist; };
		std::vector<Reading> seen;
		drive(SWEEP_MV, -SWEEP_MV);	// Clockwise
		const std::uint32_t t0 = pros::millis();
		while (lemlib::angleError(start.theta + sweep, chassis.getPose().theta, false) > 0 && pros::millis() - t0 < 3000) {
			seen.push_back({chassis.getPose().theta, inches(mount.sensor)});
			pros::delay(20);
		}
		drive(0, 0);

		std::vector<Point> objects;
		for (std::size_t i = 0; i < seen.size(); i++) {
			if (std::isnan(seen[i].dist) || seen[i].dist > maxRange) continue;
			std::size_t j = i;
			double nearest = seen[i].dist;
			while (j + 1 < seen.size() && !std::isnan(seen[j + 1].dist) && std::fabs(seen[j + 1].dist - seen[j].dist) < SCAN_JUMP)
				nearest = std::min(nearest, seen[++j].dist);
			// Middle of the run is straight at it; turning in place, the center stays at start
			const double theta = seen[(i + j) / 2].theta;
			const Point sensorAt = toField(mount.right, mount.forward, theta);
			const Point beam = toField(0, nearest + radius, theta + mount.facing);
			objects.push_back({start.x + sensorAt.x + beam.x, start.y + sensorAt.y + beam.y});
			i = j;
		}
		return objects;
	}

	// Turns to heading [deg]: planned if the feedforward is measured, else LemLib's turn
	void turnTo(double heading, int timeout = 1000) {
		if (profiled()) profiledTurn(heading);
		else chassis.turnToHeading(heading, timeout, {}, false);
	}

	// Turns in place to heading [deg] the short way, on a speed plan like profiledDrive
	void profiledTurn(double heading, double speed = 1, int timeout = 500) {
		stopMotions();
		const double delta = lemlib::angleError(heading, chassis.getPose().theta, false);
		const Trapezoid plan(delta, angularNow().topSpeed(PLAN_BATTERY) * speed, angularNow().topAccel(ACCEL_BATTERY));
		double turned = 0, w = 0, theta = chassis.getPose().theta;	// [deg], [deg/s]

		const std::uint32_t t0 = pros::millis();
		std::uint32_t wake = t0;
		while (true) {
			const double t = (pros::millis() - t0) / 1000.0;
			const double now = chassis.getPose().theta;
			const double step = std::remainder(now - theta, 360.0);
			turned += step;
			w += (step / DT - w) * SMOOTHING;
			theta = now;

			const Trapezoid::State ref = plan.at(t);
			const double e = ref.position - turned;
			if (t > plan.duration() && std::fabs(e) < SETTLED_DEG && std::fabs(w) < SETTLED_DEG_S) break;
			if (t > plan.duration() + timeout / 1000.0) {
				std::printf("profiledTurn TIMEOUT %.1f deg short\n", e);
				break;
			}
			const double push = follow(angularNow(), ref, e, w, SETTLED_DEG);
			drive(clampMv(push), clampMv(-push));
			pros::Task::delay_until(&wake, 10);
		}
		drive(0, 0);
	}

	/**
	 * Moves by an offset from the current pose, ending at heading + turn [deg].
	 * - Tank can't strafe, so a sideways offset curves there.
	 * - Chained moves stack their error; an absolute move or wallReset clears it.
	 * - Straight and blocking, it's a profiledDrive when the feedforward is measured
	 */
	void moveBy(double forward, double right = 0, double turn = 0, int timeout = 1500,
				lemlib::MoveToPoseParams params = {}, bool async = false) {
		if (profiled() && right == 0 && turn == 0 && !async) return profiledDrive(forward);
		const lemlib::Pose pose = chassis.getPose();
		const Point offset = toField(right, forward, pose.theta);
		params.forwards = forward >= 0;
		// Straight, the boomerang carrot only shrinks the error the PID sees, so it crawls in
		if (right == 0 && turn == 0) params.lead = 0;
		chassis.moveToPose(pose.x + offset.x, pose.y + offset.y, pose.theta + turn, timeout, params, async);
	}

	// Ends with the tool on target, facing heading [deg]
	void reachPose(Point target, double heading, int timeout = 2000,
				   lemlib::MoveToPoseParams params = {}, bool async = false) {
		const Point at = centerFor(target, heading);
		chassis.moveToPose(at.x, at.y, heading, timeout, params, async);
	}

	// Straight at target, backing in for a tool on the back; exact for a centerline tool
	void reachPoint(Point target, int timeout = 2000, lemlib::MoveToPointParams params = {}, bool async = false) {
		const lemlib::Pose pose = chassis.getPose();
		params.forwards = settings.toolForward >= 0;
		const double heading = bearing({pose.x, pose.y}, target) + (params.forwards ? 0 : 180);
		const Point at = centerFor(target, heading);
		chassis.moveToPoint(at.x, at.y, timeout, params, async);
	}

	// Turns the tool to face target
	void aim(Point target, int timeout = 1000, lemlib::TurnToPointParams params = {}, bool async = false) {
		params.forwards = settings.toolForward >= 0;
		chassis.turnToPoint(target.x, target.y, timeout, params, async);
	}

	// Positive is clockwise; turns that way even past 180°
	void turnBy(double degrees, int timeout = 1000, lemlib::TurnToHeadingParams params = {}, bool async = false) {
		params.direction = degrees >= 0 ? lemlib::AngularDirection::CW_CLOCKWISE : lemlib::AngularDirection::CCW_COUNTERCLOCKWISE;
		chassis.turnToHeading(chassis.getPose().theta + degrees, timeout, params, async);
	}

	/**
	 * Drives into a hard stop (wall, Toggle) and cuts power once the tracking wheel stops,
	 * so the end position doesn't depend on battery or tiles like a timed drive does.
	 * - holdMs: full-voltage push after the stop, since a soft wall bends and stops the robot early
	 */
	void pushUntilStall(int mv, int timeout = 1500, int holdMs = 0) {
		drive(mv, mv);
		const std::uint32_t start = pros::millis();
		pros::delay(settings.stallGrace);
		while (pros::millis() - start < timeout && std::abs(tracking.get_velocity()) > settings.stallVelocity)
			pros::delay(10);
		if (holdMs > 0) {
			const int full = mv < 0 ? -12000 : 12000;
			drive(full, full);
			pros::delay(holdMs);
		}
		drive(0, 0);
	}

	/**
	 * Measures where each distance mount sits (right and forward of the tracking center),
	 * so the center never has to be found by eye. Pushes into a wall to square up, backs off,
	 * then sweeps each sensor slowly across that wall both ways, spinning in place so the center stays put:
	 * - along the beam, every sensor sees the same center-to-wall distance minus its offset
	 * - sideways, a sensor off the beam's center line reads longer turning one way than the other
	 * - i, j: two mounts facing opposite ways; apart: tape-measured distance between their faces [in]
	 * - squareMv: push that squares up on a flat bumper; negative backs into the wall
	 * - Start a few inches from the wall; after backing off it needs room to spin
	 */
	std::string measureMounts(std::size_t i, std::size_t j, double apart,
							  int squareMv = -6000, double backOff = 6, double sweep = 15) {
		pushUntilStall(squareMv, 1500, 300);
		const double wall = chassis.getPose().theta + (squareMv < 0 ? 180 : 0);

		// Straight off the wall by measured distance; a PID move from a stall can overshoot far
		const int away = squareMv < 0 ? 3000 : -3000;
		const lemlib::Pose start = chassis.getPose();
		drive(away, away);
		const std::uint32_t t0 = pros::millis();
		while (chassis.getPose().distance(start) < backOff && pros::millis() - t0 < 2000) pros::delay(10);
		drive(0, 0);

		// Per mount: beam angle off the wall's normal [rad], and the reading [in].
		// A slow steady spin samples continuously; sweeping both ways cancels the sensor's lag
		std::vector<std::vector<Point>> sweeps(mounts.size());
		for (std::size_t k = 0; k < mounts.size(); k++) {
			const double square = wall - mounts[k].facing;
			chassis.turnToHeading(square - sweep - 5, 3000, {}, false);	// Up to 180° between opposite sensors
			for (const int dir : {1, -1}) {
				drive(dir * SWEEP_MV, -dir * SWEEP_MV);
				const std::uint32_t t1 = pros::millis();
				while (dir * offSquare(square) < sweep + 5 && pros::millis() - t1 < 4000) {
					const double reading = inches(mounts[k].sensor);
					const double off = offSquare(square);
					if (!std::isnan(reading) && std::fabs(off) <= sweep) sweeps[k].push_back({toRad(off), reading});
					pros::delay(20);
				}
				drive(0, 0);
				pros::delay(200);
			}
		}

		// reading·cos φ = center - along·cos φ + side·sin φ; along·(1 - cos φ) is small, so refine it over a few passes
		const std::size_t n = mounts.size();
		std::vector<double> along(n, 0), side(n, NAN), square(n, NAN);
		for (int pass = 0; pass < 3; pass++) {
			for (std::size_t k = 0; k < n; k++) {
				if (sweeps[k].size() < 3) continue;
				std::vector<Point> line;
				for (const Point& s : sweeps[k])
					line.push_back({std::sin(s.x), s.y * std::cos(s.x) - along[k] * (1 - std::cos(s.x))});
				fitLine(line, side[k], square[k]);
			}
			const double center = (apart + square[i] + square[j]) / 2;	// along[i] + along[j] = apart
			for (std::size_t k = 0; k < n; k++) along[k] = center - square[k];
		}

		std::string summary = "Mounts from center [in]:\n";
		char line[64];
		for (std::size_t k = 0; k < n; k++) {
			const double psi = toRad(mounts[k].facing);
			if (std::isnan(side[k]) || std::isnan(along[k])) std::snprintf(line, 64, "%zu: no reading (%zu samples)\n", k + 1, sweeps[k].size());
			else std::snprintf(line, 64, "%zu: right %.2f  forward %.2f\n", k + 1,
							   along[k] * std::sin(psi) + side[k] * std::cos(psi),
							   along[k] * std::cos(psi) - side[k] * std::sin(psi));
			summary += line;
		}
		std::printf("%s", summary.c_str());
		return summary;
	}

	const std::vector<DistanceMount>& getMounts() const { return mounts; }
	const OdomSettings& getSettings() const { return settings; }

private:
	static double toRad(double degrees) { return degrees * std::numbers::pi / 180; }

	// Least-squares line through points
	static void fitLine(const std::vector<Point>& points, double& slope, double& intercept) {
		double sx = 0, sy = 0, sxx = 0, sxy = 0;
		for (const Point& p : points) { sx += p.x; sy += p.y; sxx += p.x * p.x; sxy += p.x * p.y; }
		const double count = points.size();
		slope     = (count * sxy - sx * sy) / (count * sxx - sx * sx);
		intercept = (sy - slope * sx) / count;
	}

	static constexpr int SWEEP_MV = 2500;	// Slow enough for the sensor to keep up, enough to beat friction

	// NaN if unplugged or nothing in range
	static double inches(pros::Distance& sensor) {
		const std::int32_t mm = sensor.get();
		return mm == PROS_ERR || mm >= 9999 ? NAN : mm / 25.4;	// 9999 = nothing in range
	}

	static constexpr double DT            = 0.01;	// [s] control loop
	static constexpr double SCAN_JUMP     = 0.75;	// [in] reading change that means the beam moved to another object
	static constexpr double SMOOTHING     = 0.5;	// of each new speed reading, against sensor jitter
	static constexpr double PLAN_BATTERY  = 0.75;	// plans use this much of the battery for speed, leaving feedback room
	static constexpr double ACCEL_BATTERY = 0.5;	// and this much to speed up; less keeps the wheels from slipping
	static constexpr double AIM_LOCK      = 4;		// [in] from the end, stop steering at the aim point
	static constexpr double SETTLED_IN    = 0.3,  SETTLED_IN_S  = 1.5;	// [in], [in/s]
	static constexpr double SETTLED_DEG   = 0.75, SETTLED_DEG_S = 5;	// [deg], [deg/s]

	bool profiled() const { return settings.lateral.measured() && settings.angular.measured(); }
	double loadNow() const { return settings.load ? settings.load() : 0; }
	Feedforward lateralNow() const { return settings.lateral.toward(settings.lateralLoaded, loadNow()); }
	Feedforward angularNow() const { return settings.angular.toward(settings.angularLoaded, loadNow()); }

	// Voltage to follow a plan: feedforward plus feedback on position and speed error. Once the plan
	// has stopped, friction would hold a small miss, so push through it until within tolerance
	double follow(const Feedforward& ff, Trapezoid::State ref, double error, double speed, double tolerance) const {
		const Feedback fb = Feedback::forPlant(ff, settings.bandwidth);
		double mv = ff.mv(ref.velocity, ref.acceleration) + fb.kP * error + fb.kD * (ref.velocity - speed);
		if (ref.velocity == 0 && std::fabs(error) > tolerance / 2) mv += std::copysign(ff.kS, error);
		return mv;
	}

	static double clampMv(double mv) { return std::clamp(mv, -12000.0, 12000.0); }

	// A cancelled LemLib motion stops the motors as it ends, so wait it out before driving
	void stopMotions() {
		chassis.cancelAllMotions();
		chassis.waitUntilDone();
	}

	// Median of a few readings, so one stray beam doesn't count; NaN if most saw nothing
	static double medianInches(pros::Distance& sensor, int samples = 5) {
		std::vector<double> seen;
		for (int i = 0; i < samples; i++) {
			if (const double d = inches(sensor); !std::isnan(d)) seen.push_back(d);
			pros::delay(20);
		}
		if ((int)seen.size() * 2 <= samples) return NAN;
		std::nth_element(seen.begin(), seen.begin() + seen.size() / 2, seen.end());
		return seen[seen.size() / 2];
	}

	static const char* wallName(Wall wall) {
		constexpr const char* names[] = {"red", "blue", "audience", "far"};
		return names[static_cast<int>(wall)];
	}

	// Readings taken while still rolling or turning are of the wrong spot; a centered
	// tracking wheel doesn't see turning, so watch the drive too
	void waitUntilStopped(int timeout = 500) {
		const std::uint32_t start = pros::millis();
		auto moving = [&] {
			return std::abs(tracking.get_velocity()) > settings.stallVelocity ||
				   std::fabs(left.get_actual_velocity()) > STOPPED_RPM || std::fabs(right.get_actual_velocity()) > STOPPED_RPM;
		};
		do pros::delay(20);
		while (moving() && pros::millis() - start < (std::uint32_t)timeout);
	}

	static constexpr double STOPPED_RPM = 5;

	// Where the tool goes so it ends standoff short of target, coming in tool first
	Point shortOf(Point target, double heading, double standoff) const {
		const Point back = toField(0, settings.toolForward >= 0 ? standoff : -standoff, heading);
		return {target.x - back.x, target.y - back.y};
	}

	// Current heading's angle past square [deg], wrapped to ±180
	double offSquare(double square) const { return std::remainder(chassis.getPose().theta - square, 360.0); }

	void drive(int leftMv, int rightMv) {
		left.move_voltage(leftMv);
		right.move_voltage(rightMv);
	}

	// Where the tracking center goes so the tool lands on target
	Point centerFor(Point target, double heading) const {
		const Point tool = toField(settings.toolRight, settings.toolForward, heading);
		return {target.x - tool.x, target.y - tool.y};
	}

	lemlib::Chassis& chassis;
	pros::MotorGroup& left;
	pros::MotorGroup& right;
	pros::Rotation& tracking;
	const std::vector<DistanceMount> mounts;
	const OdomSettings settings;
};

} // namespace tide
