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
pros::Rotation clawEncoder(6);
pros::Imu imu(13);
pros::Distance leftDistance(3);
pros::Distance rightDistance(2);
pros::Distance middleDistance(1);

// Mechanisms
pros::Motor clawPivot(19, pros::MotorGearset::green);
pros::Motor claw(-18, pros::MotorGearset::green);
pros::MotorGroup winch({21, -20}, pros::MotorGearset::blue);
pros::Motor intakeMotor(16, pros::MotorGearset::blue);

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region ControlDefinition //////////////////////////////
////////////////////////////////////////////////////////////////

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Input curves
lemlib::ExpoDriveCurve throttle_curve(
	3, 		// joystick deadband out of 127
	10, 	// minimum output where drivetrain will move out of 127
	1.019 	// expo curve gain
);
lemlib::ExpoDriveCurve steer_curve(
	3, 		// joystick deadband out of 127
	10, 	// minimum output where drivetrain will move out of 127
	1.019 	// expo curve gain
);

// Drivetrain
lemlib::Drivetrain drivetrain(
	&leftMotors, 				// left
	&rightMotors, 				// right
	11.4173,  					// track width
	lemlib::Omniwheel::NEW_275, // wheel type
	450, 						// drivetrain rpm
	1 							// horizontal drift is 2 (for now)
);

// Odometry
lemlib::TrackingWheel vertical_tracking_wheel(&verticalEncoder, lemlib::Omniwheel::NEW_2, 0);
lemlib::OdomSensors sensors(
	&vertical_tracking_wheel,
	nullptr, nullptr, nullptr,	// unused tracking wheels
	&imu
);

// PID Tunings
lemlib::ControllerSettings lateral_controller(
	8, // prop gain		(kP)
	0 , 	// integral gain 	(kI)
	20, // derivative gain 	(kD)
	3, 	// anti windup
	1, 	// small error range 			[in]
	100,// small error ran  ge timeout	[ms]
	3, 	// large error range 			[in]
	500,// large error range timeout	[ms]
	0 	// maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(
	2,  // KP
	0,  // KI
	10,  // KD
	3, 	// anti windup
	1, 	// small error range 			[deg]
	100,// small error range timeout	[ms]
	3, 	// large error range			[deg]
	500,// large error range timeout	[ms]
	0 	// maximum acceleration (slew)  Old 30
);

/*
2.5, 	// prop gain 		(kP)
0, 	// integral gain 	(kI)
14, // derivative gain 	(kD)

*/

// Chassis definition
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

// Measured from the tracking center. Unmeasured until filled in; wall resets are wrong before then
tide::Odom odom(chassis, leftMotors, rightMotors, verticalEncoder, {
	// sensor          right [in]  forward [in]  facing
	{leftDistance,     0,          0,            WEST},
	{rightDistance,    0,          0,            EAST},
	{middleDistance,   0,          0,            NORTH},
});

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region MechanismPID ////////////////////////////////////
////////////////////////////////////////////////////////////////

// Ratios scale encoder degrees
tide::Mechanism clawPivotPID(clawPivot, clawEncoder, {
	.kP     = 200,	// [mV/deg]
	.kI     = 0,	// counteracts grav.
	.kD     = 1400,
	.windup = 0,	// [deg]
	.settle = 0,	// stops bounce [deg]
}, 1.0);

tide::Mechanism winchPID(winch, winchEncoder, {
	// .kP     = 100,
	// .kI     = 14,
	// .kD     = 400,
	// .windup = 20,
	// .settle = 5,
	.kP     = 69.04,
	.kI     = 4.4186,
	.kD     = 719.2,
	.kG		= 2025,
}, 1.0);

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Startup /////////////////////////////////////////
////////////////////////////////////////////////////////////////

void initRobot() {
	// Here, not opcontrol: field auton runs before opcontrol, so motors would still coast
	claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.calibrate();
	clawPivotPID.start();
	winchPID.start();
}

#pragma endregion
////////////////////////////////////////////////////////////////
