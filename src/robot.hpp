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

#include "main.h"			// IWYU pragma: keep
#include "lemlib/api.hpp"	// IWYU pragma: keep
#include "tide/mechanism.hpp"
#include "tide/odom.hpp"

////////////////////////////////////////////////////////////////
#pragma region Ports ///////////////////////////////////////////
////////////////////////////////////////////////////////////////

// Drivetrain
inline pros::MotorGroup	leftMotors ({  7,   9 }, pros::MotorGearset::blue);
inline pros::MotorGroup rightMotors({ -8, -10 }, pros::MotorGearset::blue);

// Sensors
inline pros::Rotation 	verticalEncoder(5);
inline pros::Rotation 	horizontalEncoder(12);
inline pros::Rotation 	winchEncoder(-14);
inline pros::Imu 	  	imu(13);
inline pros::Distance 	leftDistance(3);
inline pros::Distance 	rightDistance(2);
inline pros::Distance 	frontDistance(15);
inline pros::Distance 	backDistance(1);
inline pros::Optical  	colorSensor(6);
inline pros::Distance	clawDistance(4);

// Mechanisms
inline pros::Motor 		activeToggle(-17, pros::MotorGearset::green);
inline pros::Motor 		claw(-18, pros::MotorGearset::green);
inline pros::MotorGroup winchMotors({21, -20, -19}, pros::MotorGearset::blue);

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Mechanisms //////////////////////////////////////
////////////////////////////////////////////////////////////////

inline tide::Mechanism winch(winchMotors, winchEncoder, {
	.kP		= 67.53,
	.kI		= 4.9110,
	.kD		= 619.0,
	.kG		= 1423,	// [mV]
}, {.min = 0});	// Boots resting on bottom

constexpr double LIFT_LOADED = 3300;	// winch [deg] the "FF up" run measures at: the highest the autons carry

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Control /////////////////////////////////////////
////////////////////////////////////////////////////////////////

inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Driver curves
inline lemlib::ExpoDriveCurve throttleCurve(
	3,		// deadband		[/127]
	10,		// min output	[/127]
	1.019	// expo gain
);
inline lemlib::ExpoDriveCurve steerCurve(
	3,		// deadband		[/127]
	10,		// min output	[/127]
	1.019	// expo gain
);

inline lemlib::Drivetrain drivetrain(
	&leftMotors,
	&rightMotors,
	11.4173,					// track width [in]
	lemlib::Omniwheel::NEW_275,
	450,						// [rpm]
	8							// horizontal drift	traction middle wheels; lower caps curve speed hard
);

// Odometry
constexpr float TRACKING_WHEEL = lemlib::Omniwheel::NEW_2;	// [in] diameter
constexpr float TRACKING_WHEEL_H = lemlib::Omniwheel::NEW_275;	// [in] diameter
inline lemlib::TrackingWheel verticalWheel(&verticalEncoder, TRACKING_WHEEL, -0.01);
inline lemlib::TrackingWheel horizontalWheel(&horizontalEncoder, TRACKING_WHEEL_H, -0.01);
inline lemlib::OdomSensors sensors(
	&verticalWheel, nullptr, 
	&horizontalWheel, nullptr,
	&imu
);

// Chassis PID
inline lemlib::ControllerSettings lateralController(
	// 9,			// kP
	// 0.21324,	// kI
	// 80,			// kD
	10.048,			// kP
	0.28812,	// kI
	233.62,			// kD
	6,			// anti windup		[in] wider pushes the robot on after it should be slowing
	1,			// small error		[in]
	100,		// small timeout	[ms]
	1.5,		// large error		[in] wider counts a friction stall short of the target as arrived
	500,		// large timeout	[ms]
	0			// slew
);
inline lemlib::ControllerSettings angularController(
	1.369,		// kP
	0.05589,	// kI
	22.37,		// kD
	20,			// anti windup		[deg] integral pushes through turn friction once this close
	1,			// small error		[deg]
	100,		// small timeout	[ms]
	3,			// large error		[deg]
	500,		// large timeout	[ms]
	0			// slew
);

inline lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

// Tracking center to each bumper [in]; guesses until the Tune page's "Bumpers" run
constexpr double FRONT_BUMPER = 5.2;
constexpr double BACK_BUMPER  = 10.5;

// Offsets from the tracking center
inline tide::Odom odom(chassis, leftMotors, rightMotors, verticalEncoder, {
	// sensor			right [in]	forward [in]	facing
	{leftDistance,		-5.89,		-0.33,			tide::dir::WEST},
	{rightDistance,		 5.33,		 1.97,			tide::dir::EAST},
	{frontDistance,		-0.43,		-0.81,			tide::dir::NORTH},
}, {
	.toolForward   = -BACK_BUMPER,	// [in] claw mouth at the back bumper

	// SIM VALUES: replace with the Tune page's "FF down" / "FF up" results before running autons
	.lateral       = {1205, 166.49, 41.66},	// [mV], [mV per in/s], [mV per in/s²]
	.lateralLoaded = {1205, 166.49, 58.24},
	.angular       = {1205, 16.59, 4.15},	// per deg
	.angularLoaded = {1205, 16.59, 5.80},
	.load          = [] { return winch.position() / LIFT_LOADED; },
});

// Claw's piece sensor, looking out the back; guess: at the mouth. Measure like odom's mounts
inline const tide::DistanceMount clawMount = {clawDistance, 0, -BACK_BUMPER, 180};

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Startup /////////////////////////////////////////
////////////////////////////////////////////////////////////////

inline void initRobot() {
	claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	activeToggle.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	colorSensor.set_integration_time(20);  // Refresh rate on color sensor
	chassis.calibrate();
	winch.start();
}

#pragma endregion
////////////////////////////////////////////////////////////////
