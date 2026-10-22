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



#include "main.h"
#include "lemlib/api.hpp"	// IWYU pragma: keep
#include "tide/control.hpp"

using namespace tide::btn;



////////////////////////////////////////////////////////////////
#pragma region Ports ///////////////////////////////////////////
////////////////////////////////////////////////////////////////
const int UNDEF_PORT = 0;

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
#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region MechanismPID ////////////////////////////////////
////////////////////////////////////////////////////////////////

// Ratios scale encoder degrees
tide::Mechanism clawPivotPID(clawPivot, clawEncoder, {
	// .kP     = 400,	// [mV/deg]
	// .kI     = 3,	// counteracts grav.
	// .kD     = 800,
	.kP     = 200,	// [mV/deg]
	.kI     = 0,	// counteracts grav.
	.kD     = 800,
	.windup = 0,	// [deg]
	.settle = 0,	// stops bounce [deg]
}, 1.0);

tide::Mechanism winchPID(winch, winchEncoder, {
	.kP     = 100,
	.kI     = 14,
	.kD     = 200,
	.windup = 20,
	.settle = 5,
}, 1.0);

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region BaseFunctions ///////////////////////////////////
////////////////////////////////////////////////////////////////
/**
 * Callback function for LLEMU's center button.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) pros::lcd::set_text(2, "I was pressed!");
	else pros::lcd::clear_line(2);
}

/**
 * Initalization triggered upon execution.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);
	chassis.calibrate();
	clawPivotPID.start();
	winchPID.start();
	clawPivotPID.tare();
}

/**
 * Code lock during disabled state via Field Management System or
 * the VEX Competition Switch.
 */
void disabled() {}

/**
 * Run after initalize() before match starts.
 * Allows selection of specific autons.
 */
void competition_initialize() {}

void rot90() {
	chassis.setPose(0,0,0);
	chassis.moveToPose(0, 0, 90, 10000);
	pros::delay(1000);

}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Checks //////////////////////////////////////////
////////////////////////////////////////////////////////////////

bool isWinchDown() { return winchEncoder.get_position() < 480; }

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Macros //////////////////////////////////////////
////////////////////////////////////////////////////////////////

// Defs
const int CLAW_PIVOT_UP   = -140;	// [deg]
const int CLAW_PIVOT_DOWN = 0;	// [deg]

void clawPivotUp()   { 
	clawPivotPID.moveTo(CLAW_PIVOT_UP); 
}
void clawPivotDown() { clawPivotPID.moveTo(CLAW_PIVOT_DOWN); }

/**
 * Run when needing to score. 
 * Start: Cascade level to score, claw in Up position.
 * End: Intake position; Cascade down, claw down.
 */
void scoringMacro() {
	claw.move_voltage(-12000);  // Spit out from claw
	pros::delay(300);
	winchPID.moveBy(200); 		// Lift cascade a tad
	clawPivotPID.moveBy(-100);  // Move claw up slightly
	pros::delay(500);  			// Wait for OP drive back
	claw.move_voltage(0);  		// Stop claw
	winchPID.moveTo(0);  		// Move winch down
	clawPivotPID.moveTo(CLAW_PIVOT_DOWN); // Move claw down

	while (!isWinchDown())
		pros::delay(100);
	clawPivotPID.release();		// Drop claw
}

/**
 * Run when needing to pick a pin off of the floor.
 * Start: Cascade level to pick up, claw in up position
 * End: Same; pin in claw. 
 */
void pickupMacroPart1() {
	clawPivotPID.moveTo(CLAW_PIVOT_UP - 75);  // Lift claw
}	
void pickupMacroPart2(int pickUpTime=1000) {
	claw.move_voltage(12000);	// Spin claw
	clawPivotDown();
	pros::delay(pickUpTime);
	clawPivotUp();				// Make claw horizontal
	claw.move_voltage(0);		// Stop claw
}

 /**
 * Run when needing to pick a pin off of the floor.
 * Start: Cascade level to pick up, claw in up position
 * End: Same; pin in claw. 
 */
void clawIntakeMacro() {
	winchPID.moveTo(195);
	clawPivotPID.moveTo(50);
}

void driveStraight(int delay, int volts=6500) {
	if (delay < 0) {
		delay *= -1;
		volts *= -1;
	}

	leftMotors.move_voltage(volts);
	rightMotors.move_voltage(volts);
	pros::delay(delay);
	leftMotors.move_voltage(0);
	rightMotors.move_voltage(0);
}

#pragma endregion
////////////////////////////////////////////////////////////////

/**
 * Runs the autonomous code via Field Management System or
 * the VEX Competition Switch. May be called manually for testing.
 */
void autonomous() {
	const int LENGTH = 16;
	const int BUFF = 8;
	const int TILE = 24;

	const int NORTH = 0;
	const int SOUTH = 180;
	const int EAST 	= 90;
	const int WEST  = 270;

	const int NORTH_EAST = 45;
	const int NORTH_WEST = 270 + 45;
	const int SOUTH_EAST = 180 - 45;
	const int SOUTH_WEST = 180 + 45;

	// Fix shitty rubber band
	// winchPID.moveTo(0);

	// Suck up match load
	clawIntakeMacro();

	// Double toggle
	driveStraight(-200);
	driveStraight(500);
	driveStraight(-200);
	driveStraight(500);

	// Go to goal
	chassis.setPose(0, -65, 180);  // Reset
	clawPivotUp();
	winchPID.moveTo(60);
	chassis.moveToPose(-21, -48, 90, 2000, {.forwards = false, .minSpeed=50}, false);
	pros::delay(150);
	claw.move_voltage(-12000);
	pros::delay(230);

	// Drive away from goal w/ claw up
	claw.move_voltage(0);
	winchPID.moveBy(50);
	chassis.moveToPoint(-5, -47, 750);
	pickupMacroPart1();

	// To cuppin with claw ready
	chassis.turnToHeading(44, 500, {}, false);
	driveStraight(-1400);
	driveStraight(100);

	// Pickup cuppin
	winchPID.moveTo(0);
	pickupMacroPart2();
	claw.move_voltage(12000);
	winchPID.moveBy(400);
	
	// Turn out
	chassis.turnToHeading(45, 500, {}, false);
	driveStraight(-150);
	chassis.turnToHeading(55, 500, {}, false);
	chassis.turnToHeading(90, 500, {}, false);
	chassis.setPose(-24, -60, 90);  // Reset
	chassis.moveToPose(-12, -60, 90, 1000, {}, false);
	chassis.setPose(-12, -60, 90);  // Reset
	chassis.turnToHeading(SOUTH_EAST, 500, {}, false);
	chassis.moveToPose(-26, -45, SOUTH_EAST, 1100, {.forwards = false}, false);

	// Score
	claw.move_voltage(0);
	winchPID.moveBy(-120);
	pros::delay(230);
	claw.move_voltage(-12000);
	pros::delay(500);
	winchPID.moveBy(200);
	claw.move_voltage(0);


	////////////////////////////////////////////////

	// Go to new cuppin
	chassis.moveToPose(0, -36, 0, 2000);

	pros::delay(3000);
	winchPID.moveTo(0);
}

/**
 * Runs the  control code via Field Management System or
 * the VEX Competition Switch, or after initialize() when
 * not in competition mode.
 */
void opcontrol() {
	claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	// Def controls
	tide::Control tide(controller);
	tide.when(R).bidir(intakeMotor)
		.when(R1).onlyIf(isWinchDown).macro(clawIntakeMacro)
		.when(L).bidir(winchPID)
		.when(R).bidir(claw)
		.when(A).altmacro(pickupMacroPart1, [] { pickupMacroPart2(); })
		.when(X).altmacro(clawPivotUp, scoringMacro)
		.when(B).run([] { clawPivotPID.release(); })
		.when(LEFT).run(autonomous);
	
		while (true) {
		// Refresh
		tide.update();
		pros::delay(25);
 
		// Move
		chassis.curvature(tide.axis(LY), tide.axis(RX));
	}
}
