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
pros::Imu imu(6);
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
	0, 0, 0, 0, 0, 0
);
/*
3, 	// anti windup
	1, 	// small error range 			[in]
	100,// small error ran  ge timeout	[ms]
	3, 	// large error range 			[in]
	500,// large error range timeout	[ms]
	30 	// maximum acceleration (slew)
*/
lemlib::ControllerSettings angular_controller(
	2.5, 	// prop gain 		(kP)
	0, 	// integral gain 	(kI)
	14, // derivative gain 	(kD)
	3, 	// anti windup
	1, 	// small error range 			[deg]
	100,// small error range timeout	[ms]
	3, 	// large error range			[deg]
	500,// large error range timeout	[ms]
	0 	// maximum acceleration (slew)*/
);

// Chassis definition
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);
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
	winchEncoder.reset_position();
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

/**
 * Runs the autonomous code via Field Management System or
 * the VEX Competition Switch. May be called manually for testing.
 */
void autonomous() {
	const int LENGTH = 16;
	const int BUFF = 4;
	const int TILE = 24;

	chassis.setPose(0, -62, 0);
	// chassis.moveToPose(0, -62+LENGTH, 0, 300);
	// pros::delay(1000);
	chassis.turnToHeading(90, 200);
	// pros::delay(1000);
	// pros::delay(300);
	// chassis.moveToPose(0, -62, 180, 300);
	// pros::delay(300);
}

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Checks //////////////////////////////////////////
////////////////////////////////////////////////////////////////

bool isWinchDown() { return winchEncoder.get_position() < 250; }

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region Macros //////////////////////////////////////////
////////////////////////////////////////////////////////////////

// Defs
const int CLAW_PIVOT_UP   = -350;	// [deg]
const int CLAW_PIVOT_DOWN = -70;	// [deg]
const int CLAW_PIVOT_RPM  = 100;
const int RPM = 600;

void clawPivotUp()   { 
	winch.move_relative(100, RPM);
	pros::delay(200);
	clawPivot.move_absolute(CLAW_PIVOT_UP, RPM); 
}
void clawPivotDown() { clawPivot.move_absolute(CLAW_PIVOT_DOWN, CLAW_PIVOT_RPM); }

/**
 * Run when needing to score. 
 * Start: Cascade level to score, claw in Up position.
 * End: Intake position; Cascade down, claw down.
 */
void scoringMacro() {
	claw.move_voltage(-12000);  		// Spit out from claw
	winch.move_relative(200, 200); 		// Lift cascade a tad
	pros::delay(200);                   // Wait for claw to drop
	clawPivot.move_relative(-200, 200);  // Move claw up slightly
	pros::delay(800);  				    // Wait for OP drive back
	clawPivotDown();  					// Put claw into rest mode
	claw.move_voltage(0);  				// Stop claw
	winch.move_absolute(0, 600);  		// Move winch down
}

/**
 * Run when needing to pick a pin off of the floor.
 * Start: Cascade level to pick up, claw in up position
 * End: Same; pin in claw. 
 */
void pickupMacroPart1() {
	winch.move_relative(-200, 200);	 // Move up winch a tad
	clawPivot.move_relative(-175, CLAW_PIVOT_RPM);  // Lift claw
}	
void pickupMacroPart2() {
	claw.move_voltage(12000);	// Spin claw
	clawPivotUp();				// Make calaw horizontal
	pros::delay(850);
	claw.move_voltage(0);		// Stop claw
}

 /**
 * Run when needing to pick a pin off of the floor.
 * Start: Cascade level to pick up, claw in up position
 * End: Same; pin in claw. 
 */
void clawIntakeMacro() {
	winch.move_absolute(475, RPM);
	clawPivot.move_absolute(-50, RPM);

	pros::delay(300);

	clawPivot.move_relative(200, RPM);
	winch.move_absolute(0, RPM);
	winch.move_relative(20, RPM);
	pros::delay(100);
	winch.move_relative(20, RPM);
}

#pragma endregion
////////////////////////////////////////////////////////////////



/**
 * Runs the  control code via Field Management System or
 * the VEX Competition Switch, or after initialize() when
 * not in competition mode.
 */
void opcontrol() {
	winch.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	clawPivot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	// Def controls
	tide::Control tide(controller);
	tide.when(R).bidir(intakeMotor)
		.when(R1).onlyIf(isWinchDown).macro(clawIntakeMacro)
		.when(L).bidir(winch)
		.when(R).bidir(claw)
		.when(A).altmacro(pickupMacroPart1, pickupMacroPart2)
		.when(X).altmacro(clawPivotUp, scoringMacro)
		.when(LEFT).run(autonomous);

	while (true) {
		// Refresh
		tide.update();
		pros::delay(25);

		// Keep winch stable
		//winch.move_voltage(12000);

		printf("Winch rot: %d\n", winchEncoder.get_position());

		// Move
		chassis.curvature(tide.axis(LY), tide.axis(RX));
	}
}
