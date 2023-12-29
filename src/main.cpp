#include "main.h"

// Initializes an odometry-based chassis controller with closed and open loop
// control. Odometry allows for absolute, field-based coordinate control and
// complex motion algorithms such as pure pursuit and motion profiling.
auto chassis = ChassisControllerBuilder()
	.withMotors(
		{-1, -2, -3},
		{4, 5, 6}
	) // Adds motors with ports 1, 2, 3 on left and 4, 5, 6 on right.
	.withGains(
		{0.002, 0, 0}, // Distance gains: kP, kI, kD.
		{0.0012, 0, 0}, // Angle gains: kP, kI, kD.
		{0.00001, 0, 0} // Turn gains (helps keep chassis straight): kP, kI, kD.
	) // Adds PID control to chassis with gains specified above.
	.withDimensions(
		{AbstractMotor::gearset::blue, 4.0/3.0}, // Uses blue motor cartridge.
		{{2.75_in, 10.375_in}, // Drive wheel diameter, wheel track (distance between
			// centers of wheels.
		 imev5BlueTPR} // Blue motor cartridge encoder ticks per revolution.
	) // Specifies chassis dimensions for accurate control.
	.withOdometry() // Adds odometry to chassis, since no external encoders
		// specified, uses internal motor encoders.
	.buildOdometry(); // Returns OdomChassisController with parameters listed above.

// Initializes a 2D motion profile controller to allow the chassis to follow a series of steps which define the target velocity and angle. This enables us to use path planning and create more efficient autonomous routines.
std::shared_ptr<AsyncMotionProfileController> chassisProfileController =
	AsyncMotionProfileControllerBuilder()
		.withLimits({
						0.2, // Maximum linear speed of the chassis in m/s
						2.5, // Maximum linear acceleration of the chassis in m/s^2
						60 // Maximum linear jerk of the chassis in m/s^3
					})
		.withOutput(chassis) // Relays underlying chassis control to the ChassisController initialized above.
		.buildMotionProfileController();

Controller controller; // Creates master controller. Allows for access to digital and analog input via joysticks and buttons.

const int heightIntake = 200; // Target arm motor position for intaking
const int heightTop = 3700; // Target arm motor position for match-loading

// Initializes an array of target positions for the arm.
const int heights[2] = {heightIntake, heightTop};
// Stores current height index
int targetHeight = 0;

// Arm control buttons
ControllerButton btnUp(ControllerDigital::R1); // When button is pressed, arm moves to match-loading angle
ControllerButton btnDown(ControllerDigital::R2); // When button is pressed, arm moves to intaking angle

// Initializes a PID position controller for accurate control of arm. With this flystick design, accurate and fast movement is needed for match-efficiency and a PID controller fits this criteria.
std::shared_ptr<AsyncPositionController<double, double>> armController =
	AsyncPosControllerBuilder()
		.withMotor(7)
		.withGains({0.0013, 0, 0.00005})
		.build();

const int intakeSpeed = 150;
const int shootingVelocity = 300;

ControllerButton btnIntake(ControllerDigital::L1);
ControllerButton btnOuttake(ControllerDigital::L2);
ControllerButton btnShoot(ControllerDigital::A);

// Initializes a PID velocity controller to maintain the speed of the intake/flywheel subsystem and minimize rpm-drop from match-loading and intaking
std::shared_ptr<AsyncVelocityController<double, double>> flywheelController =
	AsyncVelControllerBuilder()
		.withMotor(8)
		.withGains(0.002, 0, 0.00001)
		.build();

bool toggle_wings = false;
pros::ADIDigitalOut wing_cylinder ('A');
ControllerButton btnWingToggle(ControllerDigital::B);

bool toggle_blocker = false;
pros::ADIDigitalOut blocker_cylinder {'B'};
ControllerButton btnBlockerToggle{ControllerDigital::right};

pros::Imu inertial(11);

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

// Displays autonomous debugging information such as the current state of the chassis.
void debug_screen()
{
	while (true)
	{
		pros::lcd::set_text(0, std::to_string(chassis->getState().x.convert(inch)));
		pros::lcd::set_text(1, std::to_string(chassis->getState().y.convert(inch)));
		pros::lcd::set_text(2, std::to_string(chassis->getState().theta.convert(degree)));
		pros::delay(10);
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
	pros::Task debugTask(debug_screen);
	inertial.reset();
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
void competition_initialize()
{
}


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
void autonomous()
{
	armController->setTarget(1000);
	chassis->setState({0_in, 0_in, 0_deg});
	chassis->turnToAngle(-90_deg);
	/*chassisProfileController->generatePath({{0_in, 0_in, 0_deg},
											{44_in, -37.5_in, -60_deg}}, "A");
	flywheelController->setTarget(125);
	chassisProfileController->setTarget("A");
	chassisProfileController->waitUntilSettled();*/
}

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
	armController->setTarget(heights[targetHeight]);
	while (true) // Main operator control loop.
	{
		// Controls the chassis using the tank drive layout where the velocity of each
		// side of the chassis is controlled by a separate joystick.
		chassis->getModel()->arcade(
			// Applies a curve to joystick input. Allows for more precise control of
			// smaller movements. Visualization and in-depth explanation can be found
			// in include/rbplib/opcontrol/util/driveCurves.hpp
			rbplib::scaledExponentialCurve(
				controller.getAnalog(ControllerAnalog::leftY), // Gets joystick value
				// from [-1, 1] of left vertical axis
				2 // Applies gain (scale) to curve function.
			),
			rbplib::scaledExponentialCurve(
				controller.getAnalog(ControllerAnalog::rightX), // Gets joystick value
				// from [-1, 1] of right vertical axis
				4 // Applies gain (scale) to curve function.
			)
		);

		// Arm control
		if (btnUp.changedToPressed() && targetHeight < 1)
		{
			targetHeight++;
			armController->setTarget(heights[targetHeight]);
		}

		else if (btnDown.changedToPressed() && targetHeight > 0)
		{
			targetHeight--;
			armController->setTarget(heights[targetHeight]);
		}

		// Flywheel controller
		if (btnIntake.isPressed())
			flywheelController->setTarget(115);
		else if (btnOuttake.isPressed())
			flywheelController->setTarget(-300);
		else if (btnShoot.isPressed())
			flywheelController->setTarget(shootingVelocity);
		else
			flywheelController->setTarget(0);

		// Wing control system
		if (btnWingToggle.changedToPressed()) // Only actuate if a rising-edge case is detected
		{
			wing_cylinder.set_value(!toggle_wings);    // When false go to true and in reverse
			toggle_wings = !toggle_wings;    // Flip the toggle to match piston state
		}

		// Blocker
		if (btnBlockerToggle.changedToPressed())
		{
			blocker_cylinder.set_value(!toggle_blocker);
			toggle_blocker = !toggle_blocker;
		}

		// Wait and give up the time we don't need to other tasks.
		// Additionally, joystick values, motor telemetry, etc. all updates every 10 ms.
		pros::delay(10);
	}
}