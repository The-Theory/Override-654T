#include "robot.hpp"
#include <algorithm>
#include <cstdio>

using tide::screen::hottest;
using tide::screen::inches;

std::string robotInfo() {
	const lemlib::Pose pose = chassis.getPose();
	char text[512];
	std::snprintf(text, sizeof(text),
		"Pose        x %.1f    y %.1f    heading %.1f°\n"
		"Distance    left %s    right %s    middle %s  [in]\n"
		"Winch       %.0f°\n"
		"Claw pivot  %.0f°    sensor absolute %.1f°\n"
		"Temps       drive %.0f    winch %.0f    claw %.0f    pivot %.0f  [°C]",
		pose.x, pose.y, pose.theta,
		inches(leftDistance).c_str(), inches(rightDistance).c_str(), inches(middleDistance).c_str(),
		winchPID.position(),
		clawPivotPID.position(), clawEncoder.get_angle() / 100.0,
		std::max(hottest(leftMotors), hottest(rightMotors)), hottest(winch), claw.get_temperature(), clawPivot.get_temperature());
	return text;
}
