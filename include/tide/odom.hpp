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
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <numbers>
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
	int    stallGrace     = 200;	// [ms] to get moving before stall checks start
	int    stallVelocity  = 11000;	// [centideg/s] tracking wheel speed counted as stopped
	double toolRight      = 0;		// [in] scoring point, from the tracking center
	double toolForward    = 0;		// [in] negative for a tool on the back
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
	 * Corrects the odom axis normal to a wall from a distance reading. Robot must be
	 * stopped with the beam on bare perimeter (not a Loader or wall Scoring Objects).
	 * Uses whichever sensor faces the wall; leaves the pose alone if none can.
	 */
	bool wallReset(Wall wall) {
		pros::delay(50);	// Sensor lags; get a reading taken after stopping

		const bool xAxis = wall == Wall::RED || wall == Wall::BLUE;
		const double sign = (wall == Wall::BLUE || wall == Wall::FAR) ? 1 : -1;
		const double wallHeading = xAxis ? (sign > 0 ? dir::EAST : dir::WEST) : (sign > 0 ? dir::NORTH : dir::SOUTH);
		const lemlib::Pose pose = chassis.getPose();

		for (const DistanceMount& mount : mounts) {
			const double beam = pose.theta + mount.facing;
			if (std::fabs(lemlib::angleError(beam, wallHeading, false)) > settings.maxResetAngle) continue;

			const double dist = inches(mount.sensor);
			if (std::isnan(dist) || dist > settings.maxResetRange) continue;

			const Point sensorAt = toField(mount.right, mount.forward, pose.theta);
			const Point beamHit  = toField(0, dist, beam);
			const double wallPos = sign * settings.halfField;
			if (xAxis) chassis.setPose(wallPos - sensorAt.x - beamHit.x, pose.y, pose.theta);
			else       chassis.setPose(pose.x, wallPos - sensorAt.y - beamHit.y, pose.theta);
			return true;
		}
		return false;
	}

	/**
	 * Moves by an offset from the current pose, ending at heading + turn [deg].
	 * - Tank can't strafe, so a sideways offset curves there.
	 * - Chained moves stack their error; an absolute move or wallReset clears it.
	 */
	void moveBy(double forward, double right = 0, double turn = 0, int timeout = 1500,
				lemlib::MoveToPoseParams params = {}, bool async = false) {
		const lemlib::Pose pose = chassis.getPose();
		const Point offset = toField(right, forward, pose.theta);
		params.forwards = forward >= 0;
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
			chassis.turnToHeading(square - sweep - 5, 1500, {}, false);
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
