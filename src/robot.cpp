#include "robot.hpp"

using namespace tide::dir;

////////////////////////////////////////////////////////////////
#pragma region Ports ///////////////////////////////////////////
////////////////////////////////////////////////////////////////

// Drivetrain
pros::MotorGroup leftMotors ({-7, -9}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({ 8, 10}, pros::MotorGearset::blue);

// Sensors
pros::Rotation verticalEncoder(-5);
pros::Rotation winchEncoder(4);
pros::Imu imu(13);
pros::Distance leftDistance(3);
pros::Distance rightDistance(2);
pros::Distance middleDistance(1);

// Mechanisms
pros::Motor claw(-18, pros::MotorGearset::green);
pros::MotorGroup winch({21, -20}, pros::MotorGearset::blue);

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Control /////////////////////////////////////////
////////////////////////////////////////////////////////////////

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Driver curves
lemlib::ExpoDriveCurve throttleCurve(
	3,		// deadband		[/127]
	10,		// min output	[/127]
	1.019	// expo gain
);
lemlib::ExpoDriveCurve steerCurve(
	3,		// deadband		[/127]
	10,		// min output	[/127]
	1.019	// expo gain
);

lemlib::Drivetrain drivetrain(
	&leftMotors,
	&rightMotors,
	11.4173,					// track width [in]
	lemlib::Omniwheel::NEW_275,
	450,						// [rpm]
	1							// horizontal drift; LemLib suggests 2 for all-omni
);

// Odometry
lemlib::TrackingWheel verticalWheel(&verticalEncoder, lemlib::Omniwheel::NEW_2, 0);
lemlib::OdomSensors sensors(
	&verticalWheel,
	nullptr, nullptr, nullptr,
	&imu
);

// Chassis PID
lemlib::ControllerSettings lateralController(
	8,		// kP
	0,		// kI
	20,		// kD
	3,		// anti windup
	1,		// small error		[in]
	100,	// small timeout	[ms]
	3,		// large error		[in]
	500,	// large timeout	[ms]
	0		// slew
);
lemlib::ControllerSettings angularController(
	2,		// kP
	0,		// kI
	10,		// kD
	3,		// anti windup
	1,		// small error		[deg]
	100,	// small timeout	[ms]
	3,		// large error		[deg]
	500,	// large timeout	[ms]
	0		// slew
);

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

// Offsets from the tracking center
tide::Odom odom(chassis, leftMotors, rightMotors, verticalEncoder, {
	// sensor			right [in]	forward [in]	facing
	{leftDistance,		0,			0,				WEST},
	{rightDistance,		0,			0,				EAST},
	{middleDistance,	0,			0,				NORTH},
});

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region MechanismPID ////////////////////////////////////
////////////////////////////////////////////////////////////////

tide::Mechanism winchPID(winch, winchEncoder, {
	.kP		= 78.13,
	.kI		= 5.3885,
	.kD		= 755.3,
	.kG		= 2055,	// [mV]
}, {.min = 0});	// Zero is the boot posture, resting on the bottom

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Startup /////////////////////////////////////////
////////////////////////////////////////////////////////////////

void initRobot() {
	// Field auton runs before opcontrol, so brake modes go here
	claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.calibrate();
	winchPID.start();
}

#pragma endregion
////////////////////////////////////////////////////////////////
