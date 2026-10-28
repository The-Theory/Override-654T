#include "robot.hpp"
#include "tide/screen.hpp"
#include <algorithm>
#include <cstdio>

using tide::screen::hottest;
using tide::screen::inches;

std::string robotInfo() {
	const lemlib::Pose pose = chassis.getPose();
	char text[512];
	std::snprintf(text, sizeof(text),
		"Pose\tx %.1f    y %.1f    heading %.1f°\n"
		"Distance\tL %s    R %s    M %s in\n"
		"Winch\t%.0f°\n"
		"Temps\tdrive %.0f    winch %.0f    claw %.0f °C",
		pose.x, pose.y, pose.theta,
		inches(leftDistance).c_str(), inches(rightDistance).c_str(), inches(middleDistance).c_str(),
		winchPID.position(),
		std::max(hottest(leftMotors), hottest(rightMotors)), hottest(winch), claw.get_temperature());
	return text;
}
