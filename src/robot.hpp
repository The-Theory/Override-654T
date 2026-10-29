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
inline pros::MotorGroup leftMotors ({-7, -9}, pros::MotorGearset::blue);
inline pros::MotorGroup rightMotors({ 8, 10}, pros::MotorGearset::blue);

// Sensors
inline pros::Rotation verticalEncoder(-5);
inline pros::Rotation winchEncoder(-4);
inline pros::Imu imu(13);
inline pros::Distance leftDistance(3);
inline pros::Distance rightDistance(2);
inline pros::Distance middleDistance(1);
inline pros::Optical colorSensor(15);

// Mechanisms
inline pros::Motor activeToggle(-17, pros::MotorGearset::green);
inline pros::Motor claw(18, pros::MotorGearset::green);
inline pros::MotorGroup winchMotors({21, -20, -19}, pros::MotorGearset::blue);

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
	1							// horizontal drift; LemLib suggests 2 for all-omni
);

// Odometry
constexpr float TRACKING_WHEEL = lemlib::Omniwheel::NEW_2;	// [in] diameter, also used by tuneLateral
inline lemlib::TrackingWheel verticalWheel(&verticalEncoder, TRACKING_WHEEL, 0);
inline lemlib::OdomSensors sensors(
	&verticalWheel,
	nullptr, nullptr, nullptr,
	&imu
);

// Chassis PID
inline lemlib::ControllerSettings lateralController(
	7.277,		// kP
	0.21324,	// kI
	165.55,		// kD
	3,			// anti windup
	1,			// small error		[in]
	100,		// small timeout	[ms]
	3,			// large error		[in]
	500,		// large timeout	[ms]
	0			// slew
);
inline lemlib::ControllerSettings angularController(
	1.276,		// kP
	0.05128,	// kI
	21.15,		// kD
	3,			// anti windup
	1,			// small error		[deg]
	100,		// small timeout	[ms]
	3,			// large error		[deg]
	500,		// large timeout	[ms]
	0			// slew
);

inline lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

// Offsets from the tracking center
inline tide::Odom odom(chassis, leftMotors, rightMotors, verticalEncoder, {
	// sensor			right [in]	forward [in]	facing
	{leftDistance,		0,			0,				tide::dir::WEST},
	{rightDistance,		0,			0,				tide::dir::EAST},
	{middleDistance,	0,			0,				tide::dir::NORTH},
}, {.toolForward = -11.5});	// [in] Claw about an inch past the back bumper; estimate, measure to refine

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Mechanisms //////////////////////////////////////
////////////////////////////////////////////////////////////////

inline tide::Mechanism winch(winchMotors, winchEncoder, {
	.kP		= 24.59,
	.kI		= 2.0930,
	.kD		= 192.6,
	.kG		= 1062,	// [mV]
}, {.min = 0});	// Boots resting on bottom

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Startup /////////////////////////////////////////
////////////////////////////////////////////////////////////////

inline void initRobot() {
	claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	activeToggle.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	colorSensor.set_integration_time(20);	// [ms] fresh hue sooner, so a fast Toggle stops on the face
	chassis.calibrate();
	winch.start();
}

#pragma endregion
////////////////////////////////////////////////////////////////
