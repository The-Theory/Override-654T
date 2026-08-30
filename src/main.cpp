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
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"



////////////////////////////////////////////////////////////////
#pragma region RobotDefinition /////////////////////////////////
////////////////////////////////////////////////////////////////
// Ports
const int UNDEF_PORT = 0;
pros::MotorGroup leftMotors ({-7, -9}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({ 8, 10}, pros::MotorGearset::blue);
pros::Rotation vertical_encoder(UNDEF_PORT);
pros::Imu imu(UNDEF_PORT);
pros::Motor clawPivot(4, pros::MotorGearset::green);

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
	2 							// horizontal drift is 2 (for now)
);

// Odometry 
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);
lemlib::OdomSensors sensors(
	&vertical_tracking_wheel, 	
	nullptr, nullptr, nullptr,	// unused tracking wheels
	&imu
);

// PID Tunings
lemlib::ControllerSettings lateral_controller(
	10, // prop gain		(kP)
	0, 	// integral gain 	(kI)
	3, 	// derivative gain 	(kD)
	3, 	// anti windup
	1, 	// small error range 			[in]
	100,// small error range timeout	[ms]
	3, 	// large error range 			[in]
	500,// large error range timeout	[ms]
	20 	// maximum acceleration (slew)
);
lemlib::ControllerSettings angular_controller(
	2, 	// prop gain 		(kP)
	0, 	// integral gain 	(kI)
	10, // derivative gain 	(kD)
	3, 	// anti windup
	1, 	// small error range 			[deg]
	100,// small error range timeout	[ms]
	3, 	// large error range			[deg]
	500,// large error range timeout	[ms]
	0 	// maximum acceleration (slew)
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
void autonomous() {}

/**
 * Runs the  control code via Field Management System or
 * the VEX Competition Switch, or after initialize() when
 * not in competition mode
 */
void opcontrol() {
    while (true) {
		pros::delay(25);

        // Get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Dual-stick arcade
        chassis.curvature(leftY, rightX);

		// Claw control
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
			clawPivot.move_voltage(12000);
    }
}
#pragma endregion
////////////////////////////////////////////////////////////////
