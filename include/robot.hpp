////////////////////////////////////////////////////////////////
// Codebase of 654T - Tsunami								  //
// Lake Travis High School, Texas, United States			  //
// VEX Override 2026-2027									  //
//															  //
// By Theo Hallgren, Ryan Koontz, and Sebastian Ditsch 		  //
// Using LemLib by Liam Teale								  //
// 															  //
// Source available on: github.com/The-Theory/Override-654T	  //
// under GPL-3.0 license									  //
////////////////////////////////////////////////////////////////


#pragma once

#include "main.h"
#include "lemlib/api.hpp"	// IWYU pragma: keep
#include "tide/mechanism.hpp"
#include "tide/odom.hpp"
#include "tide/screen.hpp"
#include <string>
#include <vector>



// Hardware & chassis, defined in robot.cpp
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Rotation verticalEncoder;
extern pros::Rotation winchEncoder;
extern pros::Rotation clawEncoder;
extern pros::Imu imu;
extern pros::Distance leftDistance;
extern pros::Distance rightDistance;
extern pros::Distance middleDistance;
extern pros::Motor clawPivot;
extern pros::Motor claw;
extern pros::MotorGroup winch;
extern pros::Motor intakeMotor;

extern pros::Controller controller;
extern lemlib::Chassis chassis;
extern tide::Odom odom;
extern tide::Mechanism clawPivotPID;
extern tide::Mechanism winchPID;

// Macros, defined in macros.cpp
bool isWinchDown();
void clawPivotUp();
void clawPivotDown();
void scoringMacro();
void pickupMacroPart1();
void pickupMacroPart2(int pickUpTime = 1000);
void clawIntakeMacro();
void tareClaw();
void doubleToggle();

// Autons & tuners, defined in autons.cpp
extern const std::vector<tide::screen::Auton> autons;
extern const std::vector<tide::screen::Tuner> tuners;
