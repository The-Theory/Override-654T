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
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>



namespace tide {

// LemLib headings, pulled in with `using namespace tide::dir;`
namespace dir {
constexpr double NORTH = 0;
constexpr double EAST  = 90;
constexpr double SOUTH = 180;
constexpr double WEST  = 270;

constexpr double NORTH_EAST = 45;
constexpr double SOUTH_EAST = 135;
constexpr double SOUTH_WEST = 225;
constexpr double NORTH_WEST = 315;
}

// Perimeter walls; field frame has +x toward blue station, +y away from audience
enum class Wall { RED, BLUE, AUDIENCE, FAR };

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
};

/**
 * Position helpers on top of a LemLib chassis that don't depend on battery or tiles:
 * distance-sensor wall resets, odom-relative straight drives, and pushes into hard stops.
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

			const double heading = lemlib::degToRad(pose.theta);
			const double beamRad = lemlib::degToRad(beam);
			const double wallPos = sign * settings.halfField;
			if (xAxis) {
				const double offset = mount.right * std::cos(heading) + mount.forward * std::sin(heading);
				chassis.setPose(wallPos - offset - dist * std::sin(beamRad), pose.y, pose.theta);
			} else {
				const double offset = -mount.right * std::sin(heading) + mount.forward * std::cos(heading);
				chassis.setPose(pose.x, wallPos - offset - dist * std::cos(beamRad), pose.theta);
			}
			return true;
		}
		return false;
	}

	// Straight along current heading; negative backs up
	void driveDist(double inches, int timeout = 1500) {
		const lemlib::Pose pose = chassis.getPose(true);
		chassis.moveToPoint(pose.x + inches * std::sin(pose.theta), pose.y + inches * std::cos(pose.theta),
			timeout, {.forwards = inches > 0}, false);
	}

	/**
	 * Drives into a hard stop (wall, Toggle) and cuts power once the tracking wheel stops,
	 * so the end position doesn't depend on battery or tiles like a timed drive does.
	 */
	void pushUntilStall(int mv, int timeout = 1500) {
		left.move_voltage(mv);
		right.move_voltage(mv);
		const std::uint32_t start = pros::millis();
		pros::delay(settings.stallGrace);
		while (pros::millis() - start < timeout && std::abs(tracking.get_velocity()) > settings.stallVelocity)
			pros::delay(10);
		left.move_voltage(0);
		right.move_voltage(0);
	}

private:
	lemlib::Chassis& chassis;
	pros::MotorGroup& left;
	pros::MotorGroup& right;
	pros::Rotation& tracking;
	const std::vector<DistanceMount> mounts;
	const OdomSettings settings;
};

} // namespace tide
