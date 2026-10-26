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
	.kD     = 1400,
	.windup = 0,	// [deg]
	.settle = 0,	// stops bounce [deg]
}, 1.0);

tide::Mechanism winchPID(winch, winchEncoder, {
	.kP     = 100,
	.kI     = 14,
	.kD     = 400,
	.windup = 20,
	.settle = 5,
}, 1.0);

#pragma endregion
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
#pragma region BaseFunctions ///////////////////////////////////
////////////////////////////////////////////////////////////////
void showAuton();
void prevAuton();
void nextAuton();

/**
 * Initalization triggered upon execution.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::register_btn0_cb(prevAuton);
	pros::lcd::register_btn2_cb(nextAuton);
	showAuton();
	// Here, not opcontrol: field auton runs before opcontrol, so motors would still coast
	claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.calibrate();
	clawPivotPID.start();
	winchPID.tare();
	winchPID.start();
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
bool downState = false;

void clawPivotUp()   { 
	clawPivotPID.moveTo(CLAW_PIVOT_UP); 
	downState = false;
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
	downState = true;

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
	if(downState) {
		winchPID.moveTo(195);
		clawPivotPID.moveTo(50);
	}
	
}

void tareClaw() {
	clawPivotPID.release();
	pros::delay(400);
	clawPivotPID.tare();
	clawPivotPID.start();
}

#pragma endregion
////////////////////////////////////////////////////////////////

/**
 * Runs the autonomous code via Field Management System or
 * the VEX Competition Switch. May be called manually for testing.
 */
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

const int CUPPIN_1 = 400;
const int SCORE_CUPPIN_1 = 120;

////////////////////////////////////////////////////////////////
#pragma region Odom ////////////////////////////////////////////
////////////////////////////////////////////////////////////////

// Field frame: origin at center, +x toward blue station, +y away from audience
const double WALL = 70.2;	// center to inside face of perimeter [in] (70.25 on metal perimeters)
enum class Wall { RED, BLUE, AUDIENCE, FAR };

// Measured from the tracking center. Unmeasured until filled in; resets are wrong before then
struct DistanceMount {
	pros::Distance& sensor;
	double right;	// [in], + is robot right
	double forward;	// [in], + is robot front
	double facing;	// [deg] clockwise from robot front
};
const DistanceMount distanceMounts[] = {
	{leftDistance,   0, 0, WEST},
	{rightDistance,  0, 0, EAST},
	{middleDistance, 0, 0, NORTH},
};

const double MAX_RESET_RANGE = 40;	// [in] sensor error is ~5% of range past 8", so far walls hurt more than help
const double MAX_RESET_ANGLE = 15;	// [deg] off-perpendicular before beam can hit wall junk instead of the wall

/**
 * Corrects the odom axis normal to a wall from a distance reading. Robot must be
 * stopped with the beam on bare perimeter (not a Loader or wall Scoring Objects).
 * Uses whichever sensor faces the wall; leaves the pose alone if none can.
 */
bool wallReset(Wall wall) {
	pros::delay(50);	// Sensor lags; get a reading taken after stopping

	const bool xAxis = wall == Wall::RED || wall == Wall::BLUE;
	const double sign = (wall == Wall::BLUE || wall == Wall::FAR) ? 1 : -1;
	const double wallHeading = xAxis ? (sign > 0 ? EAST : WEST) : (sign > 0 ? NORTH : SOUTH);
	const lemlib::Pose pose = chassis.getPose();

	for (const DistanceMount& mount : distanceMounts) {
		const double beam = pose.theta + mount.facing;
		if (std::fabs(lemlib::angleError(beam, wallHeading, false)) > MAX_RESET_ANGLE) continue;

		const std::int32_t mm = mount.sensor.get();
		if (mm == PROS_ERR || mm >= 9999) continue;	// 9999 = nothing in range
		const double dist = mm / 25.4;
		if (dist > MAX_RESET_RANGE) continue;

		const double heading = lemlib::degToRad(pose.theta);
		const double beamRad = lemlib::degToRad(beam);
		if (xAxis) {
			const double offset = mount.right * std::cos(heading) + mount.forward * std::sin(heading);
			chassis.setPose(sign * WALL - offset - dist * std::sin(beamRad), pose.y, pose.theta);
		} else {
			const double offset = -mount.right * std::sin(heading) + mount.forward * std::cos(heading);
			chassis.setPose(pose.x, sign * WALL - offset - dist * std::cos(beamRad), pose.theta);
		}
		return true;
	}
	return false;
}

// Straight along current heading; negative backs up
void driveDist(double inches, int timeout = 1500) {
	const lemlib::Pose pose = chassis.getPose(true);
	chassis.moveToPoint(pose.x + inches * std::sin(pose.theta), pose.y + inches * std::cos(pose.theta),
		timeout, {.forwards = inches > 0}, false);
}

const int STALL_GRACE    = 200;		// [ms] to get moving before stall checks start
const int STALL_VELOCITY = 11000;	// [centideg/s] tracking wheel, ~2 in/s

/**
 * Drives into a hard stop (wall, Toggle) and cuts power once the tracking wheel stops,
 * so the end position doesn't depend on battery or tiles like a timed drive does.
 */
void pushUntilStall(int mv, int timeout = 1500) {
	leftMotors.move_voltage(mv);
	rightMotors.move_voltage(mv);
	const std::uint32_t start = pros::millis();
	pros::delay(STALL_GRACE);
	while (pros::millis() - start < timeout && std::abs(verticalEncoder.get_velocity()) > STALL_VELOCITY)
		pros::delay(10);
	leftMotors.move_voltage(0);
	rightMotors.move_voltage(0);
}

const double TOGGLE_BACKOFF = 4;	// [in] tune: was a 200 ms timed back-up

// Two hits on the Toggle in front of the robot
void doubleToggle() {
	for (int i = 0; i < 2; i++) {
		driveDist(-TOGGLE_BACKOFF);
		pushUntilStall(6500);
	}
}

#pragma endregion
////////////////////////////////////////////////////////////////

const int TURN_TIMEOUT = 1000;	// [ms] safety cap; shorter ones cut turns off early on a weak battery

void x_auton() {
	chassis.setPose(0, -65, SOUTH);  // Placed flush on the audience wall, facing it
	winchPID.moveTo(195);
	doubleToggle();
	wallReset(Wall::AUDIENCE);

	// Go to goal
	clawPivotUp();
	pros::delay(200);
	winchPID.moveTo(60);
	chassis.moveToPose(-24, -48, 90, 2000, {.forwards = false, .minSpeed=50}, false);
	pros::delay(150);
	claw.move_voltage(-12000);
	pros::delay(230);

	// Drive away from goal w/ claw up
	claw.move_voltage(0);
	winchPID.moveBy(50);
	chassis.moveToPoint(-5, -47, 1500);
	pickupMacroPart1();

	// To cuppin with claw ready
	chassis.turnToHeading(42, TURN_TIMEOUT, {}, false);
	pushUntilStall(-6500);
	driveDist(2);

	// Pickup cuppin
	winchPID.moveTo(0);
	pickupMacroPart2();
	claw.move_voltage(12000);
	winchPID.moveBy(400);
	
	// Turn out
	chassis.turnToHeading(45, TURN_TIMEOUT, {}, false);
	clawPivotUp();
	driveDist(-3);
	chassis.turnToHeading(55, TURN_TIMEOUT, {}, false);
	chassis.turnToHeading(EAST, TURN_TIMEOUT, {}, false);
	wallReset(Wall::AUDIENCE);
	chassis.moveToPose(-12, -60, EAST, 1500, {}, false);
	clawPivotUp();
	chassis.turnToHeading(SOUTH_EAST, TURN_TIMEOUT, {}, false);
	chassis.moveToPose(-26, -45, SOUTH_EAST, 1100, {.forwards = false}, false);

	// Score
	claw.move_voltage(0);
	winchPID.moveTo(100);
	pros::delay(500);
	clawPivotPID.release();
	pros::delay(200);
	claw.move_voltage(-12000);
	pros::delay(500);
	winchPID.moveTo(800);
	pros::delay(500);
	claw.move_voltage(0);

	// Ready to new cuppin
	chassis.moveToPose(0, -36, 0, 2000);
	winchPID.moveTo(0);
	pros::delay(500);
	tareClaw();
	pros::delay(500);
	chassis.moveToPoint(4, -47, 1500, {}, false);

	// To new cupping with claw
	pickupMacroPart1();
	chassis.turnToHeading(-40, TURN_TIMEOUT, {}, false);
	winchPID.moveTo(200);
	pushUntilStall(-6500);
	driveDist(2);

	// Pickup
	winchPID.moveTo(0);
	pickupMacroPart2();
	claw.move_voltage(12000);
	winchPID.moveBy(400);
}

void left_base_auton() {
	chassis.setPose(-65, 0, WEST);  // Placed flush on the red wall, facing it
	pros::delay(100);
	clawIntakeMacro();
	doubleToggle();
	wallReset(Wall::RED);

	// Go to goal
	clawPivotUp();
	pros::delay(100);
	winchPID.moveTo(60);
	chassis.moveToPose(-48, -23, NORTH, 2000, {.forwards = false, .minSpeed=50}, false);
	pros::delay(150);
	claw.move_voltage(-12000);
	pros::delay(230);

	// Go to cuppin
	winchPID.moveTo(400);
	chassis.moveToPose(-48, -10, NORTH, 1000, {.minSpeed=50});
	chassis.moveToPose(-24 - 2.2, -24 + 2.2, NORTH_WEST, 1500, {.forwards = false, .minSpeed=40}, false);

	// Pickup cuppin
	clawPivotUp();
	claw.move_velocity(12000);
	pros::delay(100);
	winchPID.moveTo(20);
	pros::delay(1500);
	winchPID.moveTo(600);
	claw.move_velocity(0);

	// Score cuppin
	chassis.turnToPoint(48, 24, TURN_TIMEOUT, {}, false);
	chassis.moveToPoint(-48 + 3, -24, 600, {.forwards = false, .minSpeed=50}, false);
	winchPID.moveTo(15);
	pros::delay(400);
	claw.move_velocity(-12000);
	pros::delay(400);
	claw.move_velocity(0);
	winchPID.moveTo(600);

	// Go to new cuppin
	pros::delay(400);
	chassis.moveToPoint(-30, -24, 1500, {}, false);
	chassis.turnToPoint(-47, -48, TURN_TIMEOUT, {.forwards = false}, false);
	chassis.moveToPoint(-43 , -43, 2000, {.forwards = false}, false);

	// Pickup new cuppin
	clawPivotUp();
	pros::delay(800);
	claw.move_velocity(12000);
	winchPID.moveTo(30);
	pros::delay(1000);
	winchPID.moveTo(700);
	claw.move_velocity(0);

	// Score new cuppin
	chassis.turnToPoint(-48, -24, TURN_TIMEOUT, {.forwards = false}, false);
	chassis.moveToPoint(-48, -24 - 3, 800, {.forwards = false, .maxSpeed=100}, false);
	winchPID.moveTo(15);
	pros::delay(400);
	claw.move_velocity(-12000);
	pros::delay(800);
	claw.move_velocity(0);
	winchPID.moveTo(1000);
	pros::delay(500);

	// Back out
	chassis.moveToPoint(-48, -48, 1500, {}, false);
}

void left_auton() {
	left_base_auton();

	// Toggle
}

void skills_auton() {
	left_base_auton();
	winchPID.moveTo(200);

	// Park
	chassis.turnToPoint(-5, -5, TURN_TIMEOUT, {.forwards=false}, false);
	chassis.moveToPoint(-5, -5, 1600, {.forwards=false}, false);

	// Get new
	chassis.moveToPoint(-10, -10, 800, {}, false );
	chassis.turnToPoint(-24, 32, 2000, {}, false);
	clawIntakeMacro();
	intakeMotor.move_voltage(12000);
	claw.move_voltage(12000);
	chassis.moveToPose(-24, 32, NORTH, 2000, {}, false);
	pros::delay(4000);

	// Score
	clawPivotUp();
	winchPID.moveTo(250);
	claw.move_voltage(0);
	chassis.moveToPoint(0, 0, 2000, {.forwards=false}, false);
	claw.move_voltage(-12000);

}


////////////////////////////////////////////////////////////////
#pragma region AutonSelector ///////////////////////////////////
////////////////////////////////////////////////////////////////

// Add new routes; brain left/right buttons cycles
struct Auton { const char* name; void (*run)(); };
const Auton autons[] = {
	{"Skills", skills_auton},           
	{"X",    x_auton},
	{"Left", left_auton},
};
const int AUTON_COUNT = sizeof(autons) / sizeof(autons[0]);
int selectedAuton = 0;

void showAuton() { pros::lcd::print(1, "Auton: < %s >", autons[selectedAuton].name); }
void cycleAuton(int step) {
	selectedAuton = (selectedAuton + step + AUTON_COUNT) % AUTON_COUNT;
	showAuton();
}
void prevAuton() { cycleAuton(-1); }
void nextAuton() { cycleAuton( 1); }

void autonomous() { autons[selectedAuton].run(); }

#pragma endregion
////////////////////////////////////////////////////////////////

/**
 * Runs the  control code via Field Management System or
 * the VEX Competition Switch, or after initialize() when
 * not in competition mode.
 */
void opcontrol() {
	// Def controls
	tide::Control tide(controller);
	tide.when(R).bidir(intakeMotor)
		.when(R1).onlyIf(isWinchDown).macro(clawIntakeMacro)
		.when(L).bidir(winchPID)
		.when(R).bidir(claw)
		.when(A).altmacro(pickupMacroPart1, [] { pickupMacroPart2(); })
		.when(X).altmacro(clawPivotUp, scoringMacro)
		.when(B).run([] { clawPivotPID.release(); })
		.when(LEFT).run(autonomous)
		.when(Y).macro(tareClaw);
	
	while (true) {
		// Refresh
		tide.update();
		pros::delay(25);
 
		// Move
		chassis.curvature(tide.axis(LY), tide.axis(RX));
	}
}
