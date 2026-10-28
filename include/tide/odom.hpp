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

			const std::int32_t mm = mount.sensor.get();
			if (mm == PROS_ERR || mm >= 9999) continue;	// 9999 = nothing in range
			const double dist = mm / 25.4;
			if (dist > settings.maxResetRange) continue;

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
		left.move_voltage(mv);
		right.move_voltage(mv);
		const std::uint32_t start = pros::millis();
		pros::delay(settings.stallGrace);
		while (pros::millis() - start < timeout && std::abs(tracking.get_velocity()) > settings.stallVelocity)
			pros::delay(10);
		if (holdMs > 0) {
			const int full = mv < 0 ? -12000 : 12000;
			left.move_voltage(full);
			right.move_voltage(full);
			pros::delay(holdMs);
		}
		left.move_voltage(0);
		right.move_voltage(0);
	}

	/**
	 * Measures each distance mount's offset along its beam, so the tracking center never
	 * has to be found by eye. Turns each sensor in turn to face the wall the first one faces,
	 * spinning in place, so every reading is from the same center: they differ only by offset.
	 * - Start with mounts[0] square to a wall and ~4" from it, with room to spin
	 * - i, j: two mounts facing opposite ways; apart: tape-measured distance between their faces [in]
	 */
	std::string measureMounts(std::size_t i, std::size_t j, double apart) {
		const double wall = chassis.getPose().theta + mounts[0].facing;
		std::vector<double> readings;
		for (const DistanceMount& mount : mounts) {
			chassis.turnToHeading(wall - mount.facing, 1500, {}, false);
			readings.push_back(readInches(mount.sensor));
		}
		chassis.turnToHeading(wall - mounts[0].facing, 1500, {}, false);

		// Same center for every reading: reading + offset is the same center-to-wall distance
		const double center = readings[i] + (apart + readings[j] - readings[i]) / 2;
		std::string summary = "Mounts, along beam from center:\n";
		char line[64];
		for (std::size_t k = 0; k < mounts.size(); k++) {
			if (std::isnan(readings[k])) std::snprintf(line, 64, "%zu: no reading\n", k + 1);
			else {
				const double along = center - readings[k];
				const double rad = mounts[k].facing * std::numbers::pi / 180;
				if (std::fabs(std::sin(rad)) > 0.7) std::snprintf(line, 64, "%zu: right %.2f in\n", k + 1, along * std::sin(rad));
				else                                std::snprintf(line, 64, "%zu: forward %.2f in\n", k + 1, along * std::cos(rad));
			}
			summary += line;
		}
		std::printf("%s", summary.c_str());
		return summary;
	}

private:
	// Average of several readings, since one is noisy; NaN if out of range
	static double readInches(pros::Distance& sensor, int samples = 10) {
		pros::delay(100);	// Sensor lags; skip readings taken while turning
		double sum = 0;
		for (int n = 0; n < samples; n++) {
			const std::int32_t mm = sensor.get();
			if (mm == PROS_ERR || mm >= 9999) return NAN;	// 9999 = nothing in range
			sum += mm / 25.4;
			pros::delay(40);
		}
		return sum / samples;
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
