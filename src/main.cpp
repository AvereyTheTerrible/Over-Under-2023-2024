#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	// Initializes an odometry-based chassis controller with closed and open loop
	// control. Odometry allows for absolute, field-based coordinate control and
	// complex motion algorithms such as pure pursuit and motion profiling.
	auto chassis = ChassisControllerBuilder()
		.withMotors(
			{1, 2, 3},
			{-4, -5, -6}
		) // Adds motors with ports 1, 2, 3 on left and 4, 5, 6 on right.
		.withGains(
			{0.001, 0, 0.0001}, // Distance gains: kP, kI, kD.
			{0.001, 0, 0.0001}, // Angle gains: kP, kI, kD.
			{0.001, 0, 0.0001} // Turn gains (helps keep chassis straight): kP, kI, kD.
		) // Adds PID control to chassis with gains specified above.
		.withDimensions(
			AbstractMotor::gearset::blue, // Uses blue motor cartridge.
			{{2.75_in, 11.5_in}, // Drive wheel diameter, wheel track (distance between
			 // centers of wheels.
			 imev5BlueTPR} // Blue motor cartridge encoder ticks per revolution.
		 ) // Specifies chassis dimensions for accurate control.
		.withOdometry() // Adds odometry to chassis, since no external encoders
		// specified, uses internal motor encoders.
		.buildOdometry(); // Returns OdomChassisController with parameters listed above.

	Controller controller; // Creates master controller.

	while (true) // Main operator control loop.
	{
		// Controls the chassis using the tank drive layout where the velocity of each
		// side of the chassis is controlled by a separate joystick.
		chassis->getModel()->tank(
			// Applies a curve to joystick input. Allows for more precise control of
			// smaller movements. Visualization and in-depth explanation can be found
			// in include/rbplib/opcontrol/util/driveCurves.hpp
			rbplib::scaledExponentialCurve(
				controller.getAnalog(ControllerAnalog::leftY), // Gets joystick value
				// from [-1, 1] of left vertical axis
				8.0 // Applies gain (scale) to curve function.
			),
			rbplib::scaledExponentialCurve(
				controller.getAnalog(ControllerAnalog::rightY), // Gets joystick value
				// from [-1, 1] of right vertical axis
				8.0 // Applies gain (scale) to curve function.
			)
		);

		// Wait and give up the time we don't need to other tasks.
		// Additionally, joystick values, motor telemetry, etc. all updates every 10 ms.
		pros::delay(10);
	}
}