#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include <iostream>
// #include <sys/_intsup.h>
#include "config.h"
#include <queue>

#pragma once
#include "pros/imu.hpp"
#include <cmath>

class MockIMU : public pros::Imu
{
public:
	MockIMU(int port, double gain)
		: pros::Imu(port), imu_gain(gain) {}

	double get_rotation() const override
	{
		double raw = pros::Imu::get_rotation();
		if (raw == PROS_ERR_F)
			return NAN;
		return raw * imu_gain;
	}

private:
	double imu_gain;
};

MockIMU imu(IMU, 360.0 / 363.0);

int loadToggle = 0;
double target = 0;
double toutput = 0;
bool shouldGo = false;
int state = 0;

// // Structure to hold button label and color information
// typedef struct
// {
//     const char *label; // Button name
//     bool is_red;       // Flag to determine button color (true for red, false for blue)
// } button_info_t;

// // Array of button labels and their colors
// // True makes the button red, and false makes it blue
// button_info_t button_info[] = {
//     {PN1, PC1},
//     {PN2, PC2},
//     {PN3, PC3},
//     {PN4, PC4},
//     {PN5, PC5},
//     {PN6, PC6}};

// // the number of buttons to be created based on the array above
// #define NUM_BUTTONS (sizeof(button_info) / sizeof(button_info[0]))

// static lv_obj_t *readout_label;

// // The brain screen is 480 by 272 pixels
// // Size of the button
// int button_x = BUTTON_X;
// int button_y = BUTTON_Y;

// lv_coord_t x_start = X_START;            // Starting x-coordinate for buttons
// lv_coord_t y_start = Y_START;            // Starting y-coordinate for buttons
// lv_coord_t x_offset = button_x + OFFSET; // Horizontal offset between buttons
// lv_coord_t y_offset = button_y + OFFSET; // Vertical offset between buttons
// lv_coord_t num_columns = NUM_COLUMNS;    // Number of columns in the grid

// // Button color definitions
// #define BLUE_COLOR lv_color_hex(0x0000FF)
// #define RED_COLOR lv_color_hex(0xFF0000)
// lv_style_t blue_style;
// lv_style_t red_style;

// // Callback function for button events
// static void btn_event_cb(lv_event_t *e)
// {
//     // Check if the button was pressed
//     lv_event_code_t code = lv_event_get_code(e);
//     if (code == LV_EVENT_CLICKED)
//     {
//         // Retrieve the value associated with the button
//         int value = (intptr_t)lv_obj_get_user_data(lv_event_get_target(e));

//         // Determine the color of the button from the array
//         const char *color = button_info[value - 1].is_red ? "Red" : "Blue";

//         // Update the readout with the new value and color
//         lv_label_set_text_fmt(readout_label, "Auto: %s     Color: %s", button_info[value - 1].label, color);
//     }
// }

// // Function to abstract the creation of the button
// lv_obj_t *create_button(lv_obj_t *parent, const char *text, lv_coord_t x, lv_coord_t y, int value, bool is_red)
// {
//     lv_obj_t *btn = lv_btn_create(parent);      // Create the button
//     lv_obj_set_size(btn, button_x, button_y);   // Set button size
//     lv_obj_align(btn, LV_ALIGN_TOP_LEFT, x, y); // Position the button

//     // Create a label for the button text
//     lv_obj_t *label = lv_label_create(btn);
//     lv_label_set_text(label, text);

//     // Set the label alignment to left
//     lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_LEFT, 0);
//     // Remove any padding on the left of the button to optimize space
//     lv_obj_set_style_pad_left(btn, 3, 0);

//     // Set the button color based on the `is_red` flag from the array
//     if (is_red)
//     {
//         lv_obj_add_style(btn, &red_style, 0);
//     }
//     else
//     {
//         lv_obj_add_style(btn, &blue_style, 0);
//     }

//     // Set the value of the button
//     lv_obj_set_user_data(btn, (void *)(intptr_t)value);

//     // Attach the event callback
//     lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, NULL);
//     return btn;
// }

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg,					  // left motor group
							  &right_mg,				  // right motor group
							  10.4,						  // 12 inch track width
							  lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
							  450,						  // drivetrain rpm is 450
							  2							  // horizontal drift is 2 (for now)
);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_odom, lemlib::Omniwheel::NEW_2, -5.25); //-3.25

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_odom, lemlib::Omniwheel::NEW_2, 0.5);

lemlib::OdomSensors sensors(
	&vertical_tracking_wheel,	// vertical tracking wheel 1, set to null
	nullptr,					// vertical tracking wheel 2, set to nullptr as we have none
	&horizontal_tracking_wheel, // horizontal tracking wheel 1
	nullptr,					// horizontal tracking wheel 2, set to nullptr as we don't have a
								// second one
	&imu						// inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
											  0,  // integral gain (kI)
											  30, // derivative gain (kD)
											  0,  // anti windup
											  0,  // small error range, in inches
											  0,  // small error range timeout, in milliseconds
											  0,  // large error range, in inches
											  0,  // large error range timeout, in milliseconds
											  5	  // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3.3, // proportional gain (kP)
											  0,   // integral gain (kI)
											  33,  // derivative gain (kD)
											  0,   // anti windup
											  0,   // small error range, in inches
											  0,   // small error range timeout, in milliseconds
											  0,   // large error range, in inches
											  0,   // large error range timeout, in milliseconds
											  0	   // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve
	steer_curve(3,	  // joystick deadband out of 127
				10,	  // minimum output where drivetrain will move out of 127
				1.019 // expo curve gain0
	);

lemlib::ExpoDriveCurve
	throttle_curve(3,	 // joystick deadband out of 127
				   10,	 // minimum output where drivetrain will move out of 127
				   1.019 // expo curve gain
	);

lemlib::Chassis chassis(drivetrain,			// drivetrain settings
						lateral_controller, // lateral PID settings
						angular_controller, // angular PID settings
						sensors				// odometry sensors
											// &throttle_curve, &steer_curve
);

void intakeForward()
{
	intake_preroller.move(127);
	pros::delay(10);
	intake_hooks.move(127);
	pros::delay(10);
}

void intakeBackward()
{
	intake_preroller.move(-127);
	pros::delay(10);
	intake_hooks.move(-127);
	pros::delay(10);
}

void intakeStop()
{
	intake_preroller.move(0);
	pros::delay(10);
	intake_hooks.move(0);
	pros::delay(10);
}

bool hold = false;
bool hasRing = false;

void holdRing()
{
	// detects if ring is held or not
	// hasRing is constantly updated for if a ring is detected or not
	// set hold to true if intake should stop to hold ring when it is detected or false otherwise
	while (true)
	{
		if (ring_color.get_proximity() > 100)
		{
			if (hold)
			{
				// pros::delay(100);
				intake_hooks.move(0);
			}
			hasRing = true;
		}
		else
		{
			hasRing = false;
		}
		pros::delay(10);
	}
}

bool isRed = IS_RED;
float hue = -1;
bool sort = true;
std::queue<bool> rings;
int previousRed = 0;
int currentRed = 0;
bool hasCurrent = false;
bool hasPrevious = false;
int proximity = 0;

void detectChange()
{
	ring_color.set_led_pwm(100);
	pros::delay(10);
	while (true)
	{
		hue = ring_color.get_hue();
		// proximity = ring_color.get_proximity();

		if (sort)
		{
			if ((hue < 30) && (300 > RING_PROXIMITY))
			{
				if (!IS_RED)
				{
					// pros::delay(100); //was 200
					intakeBackward();
					pros::delay(500);
					intakeForward();
				}
				// detects previous red
				// previousRed = 2;
			}
			else if ((hue > 100) && (300 > RING_PROXIMITY))
			{
				// detects previous blue
				// previousRed = 1;
				if (IS_RED)
				{
					pros::delay(200);
					intakeBackward();
					pros::delay(200);
					intakeForward();
				}
			}
			// else
			// {
			//     previousRed = 0;
			// }

			// if ((ring_distance.get() < RING_DISTANCE_THRESHOLD))
			// {
			//     hasPrevious = true;
			// }
			// else
			// {
			//     hasPrevious = false;
			// }
			// pros::delay(10);

			// hue = ring_color.get_hue();
			// proximity = ring_color.get_proximity();

			// if ((hue < 30) && (proximity > RING_PROXIMITY))
			// {
			//     // detects current red
			//     currentRed = 2;
			// }
			// else if ((hue > 100) && (proximity > RING_PROXIMITY))
			// {
			//     // detects current blue
			//     currentRed = 1;
			// }
			// else
			// {
			//     currentRed = 0;
			// }

			// if ((ring_distance.get() < RING_DISTANCE_THRESHOLD))
			// {
			//     hasCurrent = true;
			// }
			// else
			// {
			//     hasCurrent = false;
			// }
			pros::delay(10);
		}
	}
}

// bool gotBlue = true;
// bool gotRed = false;

void colorSort()
{
	// set sort to true when you want the ring sort activated, and set it to false when it should stop

	// while (true)
	// {

	//     if (sort)
	//     {
	//         if ((previousRed == 1) && (currentRed == 2))
	//         {
	//             gotRed = true;
	//         }
	//         else if ((previousRed == 0) && (currentRed == 2))
	//         {
	//             gotRed = true;
	//         }
	//         else if ((previousRed == 2) && (currentRed == 1))
	//         {
	//             gotBlue = true;
	//         }
	//         else if ((previousRed == 0) && (currentRed == 1))
	//         {
	//             gotBlue = true;
	//         }

	//         if (hasCurrent && (!hasPrevious))
	//         {
	//             // if (!rings.empty())
	//             // {
	//             //     if (rings.front())
	//             //     {
	//             //         // sort
	//             //         pros::delay(COLOR_TIME);
	//             //         intakeBackward();
	//             //         pros::delay(200);
	//             //         intakeForward();
	//             //     }
	//             //     rings.pop();
	//             // }

	//             if (gotBlue)
	//             {
	//                 // pros::delay(50);
	//                 // intakeBackward();
	//                 // pros::delay(400);
	//                 // intakeForward();
	//                 gotBlue = false;
	//             }

	//         }
	//         pros::delay(10);
	//     }
	// }
}

// void foxglove()
// {

// 	std::cout << "foxglove" << std::endl; // flag to indicate new session
// 	pros::delay(50);
// 	while (true)
// 	{
// 		lemlib::Pose pose = chassis.getPose(); // get position of robot

// 		// round and format for effective throughput
// 		Odometry odom = {std::ceil((double)pose.x * 100.0) / 100.0, std::ceil((double)pose.y * 100.0) / 100.0, std::ceil((double)pose.theta * 100.0) / 100.0};

// 		// use helper functions to add json to cout stream
// 		Message msg{"odometry", odom};
// 		std::cout << static_cast<json>(msg) << std::flush;
// 		pros::delay(50);
// 	}
// }

// void updateController()
// {
//     while (true)
//     {
//         // prints to controller up to 3 rows (this is the max possible)
//         master.clear();

//         // up to 3 lines (0-2)
//         // use %d for integer and boolean
//         // use %f for floating point
//         // if the wrong one is used could have 0 or huge random output
//         master.print(0, 0, "target: %d", loadToggle);
//         master.print(1, 0, "angle: %f", wall_rotation.get_position());
//         master.print(2, 0, "toutput: %f", wallAngle);

//         pros::delay(50);
//     }
// }

// void wallStake()
// {
// while (true)
// {
//     if (loadToggle == 1 && shouldGo)
//     {
//         shouldGo = false;
//         wall_motor.move(127);
//         pros::delay(160);
//         // wall_motor.move(0);
//         wall_motor.brake();
//     }
//     if (loadToggle == 0 && shouldGo)
//     {
//         shouldGo = false;
//         wall_motor.move(-127);
//         pros::delay(800);
//         wall_motor.move(0);
//     }

//     if (loadToggle == 2 && shouldGo)
//     {
//         shouldGo = false;
//         wall_motor.move(127);
//     }
//     pros::delay(10);
// }

// }

// test

double wallAngle;

void wallPID()
{
	double bottom = 20;
	double load = 53;
	double score = 160;

	const double tkP = 2.5; //
	const double tkI = 0;	// 00004;//lower the more perscise
	const double tkD = 5.7; // 4larger the stronger the the kD is so response is quicker
	const double kCos = 8.5;
	double terror = 0;
	double tprevious_error = 0;
	double tintegral = 0;
	double tderivative = 0;

	while (true)
	{
		switch (state)
		{
		case 0:
			target = bottom;
			break;
		case 1:
			target = load;
			break;
		case 2:
			target = score;
			break;

		default:
			target = bottom;
			break;
		}
		wallAngle = wall_rotation.get_position() / 100;
		terror = target - wallAngle;
		tintegral += terror;
		tderivative = terror - tprevious_error;
		toutput = tkP * terror + tkI * tintegral + tkD * tderivative;
		wall_motor.move(toutput + cos(((wall_rotation.get_position() / 100) - 40) * 0.017453) * kCos);
		//  wall_motor.move(cos(((wall_rotation.get_position() / 100) - 40) * 0.017453) * kCos);
		// wall_motor.move(toutput);
		tprevious_error = terror;
		pros::delay(20);

		// PID Control Loop
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
	// wall_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// pros::delay(10);
	// wall_rotation.reset();
	// pros::delay(10);
	wall_rotation.set_position(2000);
	pros::delay(10);
	ring_color.disable_gesture();
	// pros::delay(10);
	// ring_color.set_integration_time(20);

	// // Initialize button styles for blue
	// lv_style_init(&blue_style);
	// lv_style_set_bg_color(&blue_style, BLUE_COLOR);

	// // Initialize button styles for red
	// lv_style_init(&red_style);
	// lv_style_set_bg_color(&red_style, RED_COLOR);

	// // Create the readout that displays which program is selected
	// readout_label = lv_label_create(lv_scr_act());

	// // Indicate that no program has been selected yet
	// lv_label_set_text(readout_label, "Auto: NONE");

	// // Position the readout
	// lv_obj_align(readout_label, LV_ALIGN_TOP_LEFT, 10, 10);

	// // Create buttons in a grid pattern
	// int button_count = 0;
	// for (int row = 0; row < 5; row++)
	// {
	//     for (int col = 0; col < num_columns; col++)
	//     {
	//         // Stop if all the buttons are created
	//         if (button_count >= NUM_BUTTONS)
	//         {
	//             break;
	//         }

	//         lv_coord_t x = x_start + col * x_offset; // Calculate x position
	//         lv_coord_t y = y_start + row * y_offset; // Calculate y position

	//         // Get label text and color info from the 2D array
	//         const char *label_text = button_info[button_count].label;
	//         bool is_red = button_info[button_count].is_red;
	//         int button_value = (button_count + 1);

	//         // Use the previously created button creator with the values for the current button
	//         create_button(lv_scr_act(), label_text, x, y, button_value, is_red);

	//         // Track the number of buttons created
	//         button_count++;
	//     }
	// }

	lift.extend();
	pros::delay(10);
	// doinker.extend();
	// pros::delay(10);
	chassis.calibrate(); // calibrate sensors
	pros::delay(10);
	// pros::Task controller_task(updateController); // prints to controller, comment out to get back default ui

	// cannot use if using auton selector

	pros::lcd::initialize(); // initialize brain screen
	pros::delay(10);
	pros::Task screen_task([&]()
						   {
        while (true) {
            // print robot location to the brain screen
            // pros::lcd::print(0, "vertical: %f", ((float)wall_rotation.get_position())/100.0);
            // pros::lcd::print(1, "target: %f", target);
            // pros::lcd::print(2, "toutput: %f", toutput);

            // pros::lcd::print(0, "currentRed: %i", currentRed);
            // pros::lcd::print(1, "previousRed: %i", previousRed);
            // pros::lcd::print(2,"hasCurrent: %i", hasCurrent);

            pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
            pros::lcd::print(2, "Theta: %f", ((chassis.getPose().theta) )); // heading
            // pros::lcd::print(3, "IMU HEADING: %f", imu.get_heading());

            // delay to save resources
            pros::delay(20);
        } });
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
void sawp()
{
	chassis.setPose(-59, 7, 220);
	pros::Task wallstake_task(wallPID);
	pros::delay(10);

	pros::Task holdring_task(holdRing);
	pros::delay(10);

	state += 2;
	pros::delay(1000);
	state -= 2;
	chassis.moveToPose(-24, 23, -240, 2000, {.forwards = false});
	clamp.extend();
	intakeForward();
	pros::delay(500);
	chassis.turnToPoint(-23, 50, 2000);
	chassis.moveToPoint(-23, 50, 2000);

	chassis.turnToPoint(-52, -10, 2000);
	clamp.retract();
	pros::delay(500);
	chassis.moveToPoint(-52, -10, 2000);
	hold = true;

	chassis.turnToHeading(220, 2000);
	chassis.moveToPoint(-23, -24, 2000);
	clamp.extend();
	hold = false;

	chassis.turnToPoint(-23, -47, 2000);
	chassis.moveToPoint(-23, -47, 2000);

	chassis.moveToPose(-2, -23, 40, 4000, {.maxSpeed = 60});
}
void progSkills()
{
	// ---------- INITIALIZATION (â‰ˆ1 s delay) ----------
	// Set initial robot position and retract mechanisms
	chassis.setPose(-58.263, -0.459, 90);
	lift.retract();
	// doinker.extend();
	pros::delay(10);
	intakeForward();
	pros::delay(200); // Wait 1 sec

	// Start wall PID correction asynchronously
	pros::Task wallstake_task(wallPID);
	pros::delay(10);

	// start ring hold task
	pros::Task holdring_task(holdRing);
	pros::delay(10);

	// ---------- PART 1: MOGO 1 & LADY BROWN (â‰ˆ26 s worst-case) ----------
	// Move forward along x-axis to (-47, -0.459)
	chassis.moveToPoint(-47, -0.459, 700);
	pros::delay(500);
	chassis.turnToHeading(0, 700);
	pros::delay(100);

	// MOGO 1 Sequence:
	// Approach and clamp MOGO 1

	// GET MOGO 1
	chassis.moveToPoint(-50, -20, 1800, {.forwards = false, .maxSpeed = 40});
	pros::delay(1500);
	clamp.extend(); // Clamp MOGO 1
	pros::delay(100);

	// Intake first ring on MOGO 1
	chassis.turnToHeading(70, 400);
	pros::delay(100);
	chassis.moveToPoint(-18, -22, 1000);
	pros::delay(500);

	// Move towards Lady Brown goal and stop intake
	// Turn to face (0, -42) and move there normally (no early exit needed)
	chassis.turnToPoint(0, -46, 500);
	chassis.moveToPoint(0, -46, 400, {.earlyExitRange = 3});
	// Turn to face (28, -51) but exit early to begin moving sooner
	chassis.turnToPoint(33, -48, 250, {.earlyExitRange = 5});

	// Move to (28, -51) but exit early to avoid stopping completely

	// grab lb ring
	chassis.moveToPoint(33, -48, 2000);

	pros::delay(550);

	state += 1; // Raise Lady Brown mechanism
	pros::delay(10);
	// intakeStop();

	// Move to second ring on MOGO 1
	// pros::delay(2500);
	// intake_preroller.move(127);

	// drive back
	chassis.moveToPoint(9, -42, 900, {.forwards = false});
	pros::delay(100);
	chassis.turnToHeading(180, 700);

	// Move to Lady Brown scoring position and release
	chassis.moveToPoint(3, -68, 2400, {.maxSpeed = 40});

	pros::delay(1500);
	intake_hooks.move(-50);
	pros::delay(150);
	intake_hooks.move(0);
	pros::delay(10);
	state += 1; // Score Lady Brown

	pros::delay(200);

	// Reset Lady Brown mechanism and move to next position

	intakeForward();
	pros::delay(10);

	chassis.moveToPoint(7, -47, 900, {.forwards = false});
	pros::delay(700);

	// Move to (-58, -47) facing 270Â° smoothly
	state -= 2;

	chassis.turnToHeading(270, 500);
	pros::delay(500);
	chassis.moveToPose(-45, -47, 270, 700);
	pros::delay(10);
	chassis.moveToPose(-64, -47, 270, 2500, {.maxSpeed = 60});
	pros::delay(500);

	// Score Ring 6
	chassis.moveToPose(-47, -64, 145, 1500);
	pros::delay(2000);

	// Score mogo in the corner

	chassis.moveToPose(-65, -64, 45, 1500, {.forwards = false});
	pros::delay(700);
	clamp.retract();
	// ---------- PART 2: SECOND MOGO (=25 s worst-case) ----------
	// Move towards second mogo
	pros::delay(100);
	chassis.moveToPoint(-53, -36, 700);

	// Turn to align with MOGO 2 using the back clamp
	chassis.turnToPoint(-52, 5, 600, {.forwards = false});
	chassis.moveToPoint(-52, 17, 800, {.forwards = false});
	pros::delay(100);

	// SECOND HALF++++++++++++++++++++++
	//  Approach and clamp MOGO 2
	chassis.moveToPoint(-52, 17, 1500, {.forwards = false, .maxSpeed = 60});
	pros::delay(1300);
	clamp.extend();
	pros::delay(700);

	// Move and intake rings on MOGO 2
	chassis.moveToPose(-21, 24, 90, 1700);
	pros::delay(500);
	chassis.moveToPose(-28, 54, 0, 1700);
	pros::delay(500);

	chassis.moveToPose(-75, 48, 270, 2500, {.maxSpeed = 50});
	pros::delay(500);

	chassis.moveToPose(-50, 37, 350, 1000, {.forwards = false});

	chassis.moveToPoint(-51, 64, 1000, {.maxSpeed = 80});
	pros::delay(500);

	// Score MOGO 2 in the corner
	chassis.turnToPoint(-68, 75, 700, {.forwards = false});

	chassis.moveToPoint(-75, 75, 700, {.forwards = false});
	pros::delay(100);
	clamp.retract();
	pros::delay(10);

	// ---------- PART 3: MOGO 3 & LADY BROWN (â‰ˆ27 s worst-case) ----------
	// Move to MOGO 3 and prepare for Lady Brown
	// Line up for Lady Brown and score

	// LB 2++++++++++++
	chassis.moveToPoint(-12, 50, 500, {.earlyExitRange = 3});

	chassis.moveToPoint(-5, 46, 1600, {.maxSpeed = 70});
	state += 1;
	pros::delay(500);
	chassis.turnToHeading(0, 700);
	chassis.moveToPoint(-10, 70, 1800, {.maxSpeed = 40});
	pros::delay(1800);
	intake_hooks.move(-50);
	pros::delay(150);
	intake_hooks.move(0);
	pros::delay(10);
	state += 1;
	pros::delay(900);

	chassis.moveToPoint(0, 46, 1000, {.forwards = false});
	state -= 2;
	pros::delay(10);
	hold = true;
	pros::delay(300);
	intakeForward();
	// pros::delay(500);

	// FIRST AND SECOND RING MOGO 3
	chassis.moveToPoint(27, 52, 1500); //(Ring 1 mogo 3)
	pros::delay(250);
	chassis.moveToPoint(20, 18, 1500);
	// intake 2 rings and move to Mogo 3

	// chassis.moveToPoint(24, 47, 1500); //(Ring 1 mogo 3)
	// pros::delay(250);

	// chassis.moveToPose(24, 47, 130, 2000); // ring 1

	// pros::delay(500);

	// chassis.moveToPose(24, 23, 180, 2000);
	// pros::delay(500);

	// chassis.turnToHeading(320, 2000);

	// pros::delay(50);

	chassis.moveToPoint(39, 5, 2000, {.forwards = false, .maxSpeed = 60});
	pros::delay(1800);
	clamp.extend();
	pros::delay(1000);
	hold = false;
	intakeForward();

	// Intake rings on MOGO 3
	chassis.moveToPose(44, 68, 0, 2500, {.maxSpeed = 80});
	pros::delay(2000);

	// Score MOGO 3 in the corner
	chassis.turnToHeading(240, 800, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
	pros::delay(800);
	clamp.retract();
	chassis.moveToPoint(62, 68, 700, {.forwards = false});
	pros::delay(250);

	// ---------- ALLIANCE STAKE & HANG (â‰ˆ5 s worst-case) ----------
	// Move to intake rings for alliance stake
	hold = true;
	pros::delay(10);

	chassis.moveToPoint(63, 48, 2000); // intake ring for alliance
	pros::delay(500);
	chassis.moveToPoint(47, 31, 2000); // move to position for alliance

	// LINE UP ALLIANCE
	chassis.moveToPoint(47, 0, 2000); // move to position for alliance
	pros::delay(500);
	// chassis.moveToPoint(62, 0, 2000); // turn to alliance
	// pros::delay(500);
	chassis.turnToHeading(270, 800);
	chassis.moveToPoint(62, 3, 2000, {.forwards = false}); // go to alliance
	pros::delay(1500);

	hold = false;
	intakeForward();
	pros::delay(500);
	intakeStop();
	chassis.moveToPoint(56, -19, 1000, {.forwards = false}); // aligh wiht mogo
	pros::delay(500);
	chassis.moveToPoint(65, -65, 2000, {.forwards = false}); // push mogo in cornor
	// pros::delay(500);
	// state += 2; // initiate hang
	// chassis.moveToPoint(12, -14, 2000);
	pros::delay(10000);
}

void autonomous()
{
	pros::Task wallstake_task(wallPID);
	pros::delay(10);
	pros::Task hold_task(holdRing);
	pros::delay(10);
	pros::Task detect_task(detectChange);
	pros::delay(10);
	progSkills();
	// RED ringside sawp
	// auton
	// alliance stake
	// chassis.setPose(0, 0, 0);

	// pros::delay(100);
	// // chassis.moveToPoint(0, 12, 2000);
	// while (true)
	// {

	//     chassis.moveToPoint(0, 12, 2000);
	//     pros::delay(4000);
	//     chassis.moveToPoint(0, 0, 2000, {.forwards = false});
	// }

	// chassis.moveToPoint(0, 0, 8000, );
	// lift.retract();
	// doinker.extend();
	// pros::delay(500);
	// // chassis.turnToHeading(210,1000);
	// pros::delay(1000);
	// pros::Task wallstake_task(wallPID);
	// pros::delay(10);
	// chassis.moveToPose(-65, -3, 315, 2000, {.lead = 0.14, .maxSpeed = 80});
	// pros::delay(2000);
	// // wall stake movement
	// state += 2;
	// pros::delay(1000);
	// state -= 2;
	// pros::delay(1000);
	// chassis.moveToPoint(-53.5, -33, 2000, {.forwards = false, .maxSpeed = 60});
	// pros::delay(1000);
	// intakeForward();
	// pros::delay(10);
	// // Ring 1
	// // intake lift up
	// chassis.turnToPoint(-31, -36, 2000, {.forwards = false, .direction = AngularDirection::CW_CLOCKWISE});
	// pros::delay(2000);
	// chassis.moveToPoint(-31, -36, 3000, {.forwards = false, .maxSpeed = 30});
	// pros::delay(2000);
	// clamp.extend();
	// pros::delay(500);
	// chassis.turnToHeading(0, 1000);
	// pros::delay(1000);
	// chassis.moveToPoint(-24, -70, 2000, {.maxSpeed = 60});
	// pros::delay(2000);
	// chassis.turnToPoint(-2, 50, 2000, {.maxSpeed = 60});
	// pros::delay(2000);
	// chassis.moveToPoint(-2,50,2000, {.maxSpeed = 60});
	// pros::delay(4000);
	// move to ladder
	// chassis.moveToPose(5,35,180, 2000,{.lead = 0.14, .maxSpeed = 80});
	pros::delay(100000);

	// // chassis.moveToPoint(-47, 23, 2000, {.forwards = false, .maxSpeed = 60});
	// // pros::delay(500);

	// chassis.turnToHeading(180, 2000);
	// pros::delay(2000);
	// intakeForward();
	// lift.extend(); // lift intake to get top ring
	// pros::delay(2000);
	// chassis.moveToPoint(-47, 8, 2000, {.maxSpeed = 60});
	// pros::delay(100);
	// intakeStop();
	// lift.retract(); //lift intake to get top ring
	// pros::delay(500);
	// // mogo 1 + score ring 1
	// chassis.turnToHeading(240, 2000);
	// chassis.moveToPoint(-58.5, 35.5, 2000, {.forwards = false, .maxSpeed = 60});
	// // chassis.moveToPose(-32.5, 18.5, 240, 2000, {.forwards = false, .maxSpeed = 60});
	// pros::delay(500);
	// clamp.extend();
	// pros::delay(500);
	// intakeForward();
	// pros::delay(500);
	// // ring 2
	// chassis.turnToHeading(16, 2000);
	// pros::delay(500);
	// chassis.moveToPoint(-25.25, 40, 2000, {.maxSpeed = 60});
	// pros::delay(500);
	// // Touch ladder
	// chassis.turnToHeading(180, 2000);
	// pros::delay(500);
	// chassis.moveToPoint(-26, 14, 2000, {.maxSpeed = 60});
	// pros::delay(500);
	// intakeStop();
	// pros::delay(500);
	// pid turn PID
	// lift.retract();
	// pros::delay(10);

	// pros::delay(10);
	// chassis.turnToHeading(90,3000);
	// pros::delay(1000);
	// chassis.turnToHeading(180,3000);

	// el
	// ims right side
	//  chassis.setPose(0, 0, 0);
	//  pros::delay(10);
	//  lift.retract();
	//  pros::delay(10);
	//  chassis.moveToPoint(0, -40, 5000, {.forwards = false, .maxSpeed = 50});
	//  pros::delay(5000);
	//  clamp.extend();
	//  pros::delay(1000);
	//  intakeForward();
	//  chassis.turnToHeading(280, 3000);
	//  pros::delay(3000);
	//  chassis.moveToPoint((-28*1.2), (-22), 3000);

	// elims left side
	// chassis.setPose(0, 0, 0);
	// pros::delay(10);
	// lift.retract();
	// pros::delay(10);
	// chassis.moveToPoint(0, -40, 5000, {.forwards = false, .maxSpeed = 40});
	// pros::delay(5000);
	// clamp.extend();
	// pros::delay(1000);
	// intakeForward();

	// chassis.turnToHeading(-280, 3000);
	// pros::delay(3000);
	// chassis.moveToPoint((28 * 1.2), (-22), 3000);
	// pros::delay(3000);
	// chassis.turnToHeading(150, 2000, {.maxSpeed = 80});
	// pros::delay(2000);
	// chassis.moveToPoint(44, -60, 5000, {.maxSpeed = 60});

	// prog skills

	// prog skills

	pros::delay(10);
	chassis.moveToPoint(-47, -1.3, 2000, {.maxSpeed = 60}); // move to mogo goal 1
	pros::delay(2000);
	chassis.turnToHeading(180, 2000);
	pros::delay(4000);
	chassis.moveToPoint(-10, -60, 2000, {.maxSpeed = 60});
	// ring 8, 9
	pros::delay(4000);
	chassis.turnToHeading(270, 2000);
	pros::delay(4000);
	chassis.moveToPoint(-70, -60, 2000, {.maxSpeed = 60});
	pros::delay(4000);
	chassis.moveToPoint(-60, -60, 2000, {.forwards = false, .maxSpeed = 60});
	pros::delay(4000);
	chassis.turnToHeading(45, 2000);
	chassis.turnToHeading(180, 2000, {.maxSpeed = 40});
	pros::delay(2000);
	chassis.moveToPoint(-54, 24, 2000, {.forwards = false, .maxSpeed = 30});
	pros::delay(2000);
	clamp.extend(); // clamp
	pros::delay(1000);
	chassis.turnToHeading(90, 2000); // turn to ring
	// Mogo 1 Ring 1
	chassis.moveToPoint(-19, 22, 2000, {.maxSpeed = 60}); // ring 1
	pros::delay(1000);
	// add for one ring
	//         chassis.turnToHeading(135,2000);
	//         chassis.moveToPoint(-67, 59,4000, {.forwards = false, .maxSpeed = 60});
	//         clamp.retract();
	//         pros::delay(1040);
	//         chassis.moveToPoint(-55,43,1000, {.maxSpeed = 40});
	//         pros::delay(1000);

	//    // when PID is tuned
	//   /*
	// add for one ring

	chassis.turnToHeading(350, 2000, {.maxSpeed = 30}); // turn to ring 2
	pros::delay(2000);
	chassis.moveToPoint(-26, 52, 2000, {.maxSpeed = 40}); // move to ring 2
	pros::delay(2000);
	chassis.turnToHeading(270, 2000, {.maxSpeed = 40}); // turn to ring 3 and 4
	pros::delay(500);
	chassis.moveToPoint(-73, 48, 3000, {.maxSpeed = 60}); // move to ring 3 and 4
	pros::delay(2000);
	chassis.turnToHeading(45, 2000); // turn ring 5
	pros::delay(2000);
	chassis.moveToPoint(-50, 62, 2000, {.maxSpeed = 40}); // move to ring 5 //This was originally 45 ms time so change back if something broke
	pros::delay(2000);
	chassis.turnToHeading(135, 2000, {.maxSpeed = 40});
	pros::delay(2000);
	clamp.retract();														 // deposit
	pros::delay(500);														 // turn
	chassis.moveToPoint(-77, 74, 2000, {.forwards = false, .maxSpeed = 80}); // move
	pros::delay(2000);
	// from this point, all of these coordinates are guesses
	chassis.moveToPoint(-59, 66, 2000, {.forwards = false, .maxSpeed = 80});
	pros::delay(4000);
	chassis.turnToHeading(0, 2000);
	pros::delay(4000);
	chassis.moveToPoint(-60, -30, 5000, {.maxSpeed = 80});

	pros::delay(4000);
	// // // mogo 2
	chassis.turnToHeading(300, 2000);
	pros::delay(4000);
	chassis.moveToPoint(-50, -16, 2000, {.forwards = false, .maxSpeed = 60});
	pros::delay(4000);
	clamp.extend();
	pros::delay(500);
	// ring 6
	chassis.turnToHeading(90, 2000);
	pros::delay(4000);
	chassis.moveToPoint(-10, -16, 2000, {.maxSpeed = 60});
	pros::delay(4000);
	// ring 7

	//  pros::delay(2000);
	// chassis.moveToPose(-40, 56, 110, 2000, {.maxSpeed = 40});
	// pros::delay(2000);
	//  chassis.turnToHeading(55, 2000);
	//  pros::delay(1000);
	//  chassis.moveToPoint(-47, 51, 2000, {.forwards = false, .maxSpeed = 40});
	//  pros::delay(1000);
	//  chassis.turnToHeading(0, 1000);
	//  pros::delay(1000); chassis.setPose(-58.3, -1.3, 90); // start
	// pros::delay(200);
	// intakeForward(); // alliance
	// pros::delay(1000);
	// lift.retract

	//  chassis.moveToPoint(-47, -24, 2000, {.maxSpeed = 40});
	//  pros::delay(1000);
	//  clamp.extend();
	//  // ring 1
	//  chassis.turnToHeading(90, 1000);
	//  pros::delay(1000);
	//  chassis.moveToPoint(-17, -24, 1000, {.maxSpeed = 60});
	//  pros::delay(1000);
	//  // ring 2
	// chassis.turnToHeading(200, 1000);
	// pros::delay(1000);
	// chassis.moveToPoint(-24, -47, 1000, {.maxSpeed = 60});
	// pros::delay(1000);
	//  // ring 3&4
	//  chassis.turnToHeading(270, 1000);
	// pros::delay(1000);
	//  chassis.moveToPoint(-58, -47, 1000, {.maxSpeed = 70});
	//  pros::delay(1000);
	//  // ring 5
	//  chassis.turnToHeading(135, 1000);
	//  pros::delay(1000);
	//  chassis.moveToPoint(-48, -58, 1000, {.maxSpeed = 40});
	//  pros::delay(1000);
	//  // deposit mogo
	//  chassis.turnToHeading(70, 1000);
	// // pros::delay(1000);
	// chassis.moveToPoint(-50.5, -59, 1000, {.maxSpeed = 40});
	// pros::delay(1000);
	// clamp.retract();
	// chassis.moveToPoint(-3, -59, 2000, {.maxSpeed = 40});

	// set position to x:0, y:0, heading:0
	// chassis.setPose(58.5, 12.5, 0);
	// pros::delay(10);
	// chassis.moveToPoint(54, -16, 3000, {.forwards=false});
	// chassis.turnToHeading(340, 2000);
	// pros::delay(10);
	// lift.extend();
	// pros::delay(10);
	// intake_preroller.move(127);
	// chassis.moveToPoint(47, -9, 2000, {.maxSpeed = 40});
	// pros::delay(2000);
	// lift.retract();
	// pros::delay(1000);
	// chassis.turnToHeading(270, 1000, {.maxSpeed = 60});
	// chassis.moveToPoint(57, 0, 2000, {.forwards = false, .maxSpeed = 60});

	// two ring right side
	// chassis.setPose(60, 47, 270);
	// pros::delay(10);
	// lift.retract();
	// pros::delay(10);
	// intake_preroller.move(127);
	// pros::delay(10);
	// chassis.moveToPoint(20, 47, 2000, {.maxSpeed = 60});
	// chassis.turnToHeading(180, 2000, {.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
	// pros::delay(2000);
	// chassis.moveToPoint(30, 28, 2000, {.forwards = false, .maxSpeed = 50});
	// pros::delay(2000);
	// clamp.extend();
	// pros::delay(10);
	// intakeForward();
	// pros::delay(3000);
	// chassis.moveToPoint(30, 20, 5000, {.forwards = false, .maxSpeed = 40});

	// //two ring left side
	/*
	chassis.setPose(60, -47, 270);
	pros::delay(10);
	lift.retract();
	pros::delay(10);
	intake_preroller.move(127);
	pros::delay(10);
	chassis.moveToPoint(20, -47, 2000, {.maxSpeed = 60});
	chassis.turnToHeading(180, 2000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
	pros::delay(2000);
	chassis.moveToPoint(30, -28, 2000, {.forwards = false, .maxSpeed = 50});
	pros::delay(2000);
	clamp.extend();
	pros::delay(10);
	intakeForward();
	pros::delay(3000);
	intakeStop();
	pros::delay(10);
	chassis.moveToPoint(41, -6.5, 5000, {.maxSpeed = 60});
	lift.extend();
	pros::delay(10);
	intake_preroller.move(127);
	chassis.moveToPoint(30, -20, 5000, {.forwards = false, .maxSpeed = 40});


	*/
	pros::delay(10000);
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

// void ladyBrown()
// {

//     bool full = true;
//     while (true)
//     {

//         if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
//         {
//             if (wallState == 0)
//             {
//                 wall_stake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//                 wall_stake_motor.move(30);
//                 pros::delay(225);
//                 wall_stake_motor.move(0);
//                 wallState += 1;
//             }
//             else if (wallState == 1)
//             {

//                 wall_stake_motor.move(127);

//                 wallState += 1;
//             }
//             else if (wallState == 2)
//             {
//                 wall_stake_motor.move(-127);
//                 wallState = 0;
//                 pros::delay(1500);
//                 wall_stake_motor.move(0);
//             }
//         }

//         pros::delay(25);
//     }
// }
void progSkillssss()
{
	// ---------- INITIALIZATION (≈1 s delay) ----------
	// Set initial robot position and retract mechanisms
	chassis.setPose(-58.263, -0.459, 90);
	lift.retract();
	doinker.extend();
	intakeForward();
	pros::delay(1000); // Wait 1 sec

	// Start wall PID correction asynchronously
	pros::Task wallstake_task(wallPID);
	pros::delay(10);

	// start ring hold task
	pros::Task holdring_task(holdRing);
	pros::delay(10);

	// ---------- PART 1: MOGO 1 & LADY BROWN (≈26 s worst-case) ----------
	// Move forward along x-axis to (-47, -0.459)
	chassis.moveToPoint(-47, -0.459, 1000);
	pros::delay(500);

	// MOGO 1 Sequence:
	// Approach and clamp MOGO 1
	chassis.moveToPose(-47, -23, 180, 2000, {.forwards = false, .maxSpeed = 60});
	pros::delay(500);
	clamp.extend(); // Clamp MOGO 1
	pros::delay(500);

	// Intake first ring on MOGO 1
	chassis.moveToPoint(-23, -23, 2000);
	pros::delay(1000);

	// Move towards Lady Brown goal and stop intake
	state += 1; // Raise Lady Brown mechanism
	chassis.moveToPose(23, -47, 120, 2000);
	pros::delay(500);
	intakeStop();

	// Move to second ring on MOGO 1
	pros::delay(500);
	intake_preroller.move(127);
	chassis.moveToPose(-0.153, -58, 240, 2000);
	pros::delay(500);

	// Move to Lady Brown scoring position and release
	chassis.moveToPose(-0.153, -69, 180, 2000);
	pros::delay(500);
	state += 1; // Score Lady Brown
	intake_preroller.move(0);
	pros::delay(500);

	// Reset Lady Brown mechanism and move to next position
	state -= 2;
	chassis.moveToPose(-5.199, -47, 160, 2000, {.forwards = false});
	intakeForward();
	pros::delay(500);

	// Move to (-58, -47) facing 270° smoothly
	chassis.moveToPose(-58, -47, 270, 2000, {.forwards = false});
	pros::delay(500);

	// Score Ring 6
	chassis.moveToPose(-47, -58, 145, 2000);
	pros::delay(250);

	// Score mogo in the corner
	clamp.retract();
	chassis.moveToPose(-65, -64, 45, 2000, {.forwards = false});

	// ---------- PART 2: SECOND MOGO (=25 s worst-case) ----------
	// Move towards second mogo
	chassis.moveToPoint(-47, -36, 2000);

	// Turn to align with MOGO 2 using the back clamp
	chassis.turnToPoint(-47, 13, 2000, {.forwards = false});
	chassis.moveToPose(-47, 13, 180, 2000, {.forwards = false});
	pros::delay(1000);

	// Approach and clamp MOGO 2
	chassis.moveToPoint(-47, 23, 2000, {.forwards = false, .maxSpeed = 60});
	pros::delay(500);
	clamp.extend();
	pros::delay(500);

	// Move and intake rings on MOGO 2
	chassis.moveToPose(-23, 25, 90, 2000);
	pros::delay(500);
	chassis.moveToPose(-23, 47, 0, 2000);
	pros::delay(500);
	chassis.moveToPose(-58, 47, 270, 2000);
	pros::delay(500);
	chassis.moveToPose(-47, 58, 45, 2000);
	pros::delay(500);

	// Score MOGO 2 in the corner
	clamp.retract();
	chassis.moveToPose(-64, 64, 120, 2000);
	pros::delay(500);

	// ---------- PART 3: MOGO 3 & LADY BROWN (≈27 s worst-case) ----------
	// Move to MOGO 3 and prepare for Lady Brown
	chassis.moveToPose(-40, 58, 90, 2000);
	state += 1;
	pros::delay(500);

	// Line up for Lady Brown and score
	chassis.moveToPose(-0.153, 59, 0, 2000);
	pros::delay(500);
	state += 1;
	pros::delay(500);
	state -= 2;
	pros::delay(10);
	hold = true;
	pros::delay(500);

	// intake 2 rings and move to Mogo 3

	chassis.turnToPoint(24, 47, 2000); //(Ring 1 mogo 3)
	pros::delay(250);

	chassis.moveToPose(24, 47, 130, 2000); // ring 1

	pros::delay(500);

	chassis.moveToPose(24, 23, 180, 2000);
	pros::delay(500);

	chassis.turnToHeading(320, 2000);

	pros::delay(50);

	chassis.moveToPose(47, 0.153, 320, 2000, {.forwards = false, .maxSpeed = 60});
	pros::delay(250);
	clamp.extend();
	pros::delay(10);
	hold = false;
	intakeForward();

	// Intake rings on MOGO 3
	chassis.moveToPose(47, 59, 0, 2000);
	pros::delay(500);

	// Score MOGO 3 in the corner
	chassis.turnToHeading(270, 2000, {.direction = AngularDirection::CW_CLOCKWISE});
	pros::delay(500);
	clamp.retract();
	chassis.moveToPose(65, 64, 270, 2000);
	pros::delay(250);

	// ---------- ALLIANCE STAKE & HANG (≈5 s worst-case) ----------
	// Move to intake rings for alliance stake
	hold = true;
	pros::delay(10);

	chassis.moveToPose(59, 47, 130, 2000); // intake ring for alliance
	pros::delay(500);

	chassis.moveToPoint(45, 24, 2000); // move to position for alliance
	pros::delay(500);
	chassis.moveToPoint(62, 0, 2000); // turn to alliance
	pros::delay(500);
	chassis.moveToPose(62, 0, 270, 2000); // go to alliance
	pros::delay(500);

	hold = false;
	intakeForward();
	pros::delay(500);
	intakeBackward();
	chassis.moveToPose(56, -19.2, 350, 2000); // aligh wiht mogo
	pros::delay(500);
	chassis.moveToPose(65, -65, 330, 2000); // push mogo in cornor
	pros::delay(500);
	state += 2; // initiate hang
	chassis.moveToPoint(12, -14, 2000);
}

bool intake = false;
bool outake = false;
bool doinkerToggle = false;
bool rushToggle = false;
bool prerollerToggle = false;
int cooldown = 0;
bool check = false;

void opcontrol()
{
	pros::Task wallstake_task(wallPID);
	pros::delay(10);
	// pros::Task detect_task(detectChange);
	// pros::delay(10);
	// pros::Task sort_task(colorSort);
	// pros::delay(10);
	lift.retract();

	// // pros::Task controller_task(updateController); // prints to controller, comment out to get back default ui
	// pros::delay(10);
	// // lv_timer_handler(); // Process LVGL tasks
	// // pros::Task foxglove_task(foxglove);

	// // pros::Task holdring_trak(holdRing);

	// // pros::Task colorsort_task(colorSort);
	// pros::delay(10);
	// // pros::Task turnpid_task(TurnPid);
	// pros::delay(10);
	// sort = true;
	// while (true)
	// {
	//     ExampleStruct payload{x};
	//     Message msg{"my_topic_name", payload};
	//     std::cout << static_cast<json>(msg) << std::flush;

	//     pros::delay(50);
	//     x++;
	// }

	while (true)
	{
		// get left y and right x positions of joystick
		int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		// move the robot
		chassis.arcade(leftY, rightX);

		// buttons

		// when a is pressed, toggle between intaking and stopping the intake
		// overrides outtaking when pressed
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
		{
			intake = !intake;
			outake = 0;
			if (intake)
			{
				intakeForward();
			}
			else
			{
				intakeStop();
			}
		} // activate intake

		// when b is pressed, toggle between outtaking and stopping the intake
		// overrides intaking when pressed
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
		{
			outake = !outake;
			intake = 0;
			if (outake)
			{
				intakeBackward();
			}
			else
			{
				intakeStop();
			}
		}

		// when r1 is pressed, moves wall stake mechanism forward by one state
		// unless wall stake mechanism is already fully extended
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
		{

			if (state == 1)
			{
				intake_hooks.move(-50);
				pros::delay(150);
				intake_hooks.move(0);
				pros::delay(10);
				if (intake)
				{
					intake = false;
				}
			}
			if (state != 2)
			{
				state++;
			}
		}

		// when r2 is pressed, moves wall stake mechanism backward by one state
		// unless wall stake mechanism is already fully retracted
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
		{
			if (state != 0)
			{
				state--;
			}
			if (!intake)
			{
				cooldown = 20;
				check = true;
			}
		}

		if (check)
		{
			if (cooldown == 0)
			{
				check = false;
				intakeForward();
				intake = true;
			}
			cooldown--;
		}

		// extend clamp on press
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
		{
			clamp.extend();
		}

		// retract clamp on press
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
		{
			clamp.retract();
		}

		// disable color sort, for matches where colored lighting may mess up detection
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
		{
			sort = false;
		}

		// toggle the clamp on the rush arm to grab or release a mobile goal
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{

			rushToggle = !rushToggle;
			if (rushToggle)
			{
				rush.extend();
			}
			else
			{
				rush.retract();
			}
		}

		// quickly pulse the conveyor hooks backward, moving them out of the way
		// this is necessary for the hooks not to hit the ring when attempting to extend wall stake mechanism
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			intake_hooks.move(-127);
			pros::delay(50);
			intake_hooks.move(0);
			pros::delay(10);
		}

		// only toggles the preroller stage of intake
		// allows for holding one ring while intaking another
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
			prerollerToggle = !prerollerToggle;

			if (prerollerToggle)
			{
				intake_preroller.move(127);
			}
			else
			{
				intake_preroller.move(0);
			}
		}

		// toggle the goal arm
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
		{
			doinkerToggle = !doinkerToggle;
			if (doinkerToggle)
			{
				doinker.extend();
			}
			else
			{
				doinker.retract();
			}
		}

		pros::delay(20);
	}
}

/*
bool ejectRing()
{
	// optical sensor
	double hue = Optic.get_hue();
	bool redOrBlu = 0; // 0 for red, 1 for blue

	if (std::min(hue, 360 - hue) > abs(hue - 180))
	{
		// distance from 0 or 360     //distance from 180
		redOrBlu = 1;
		// if the distance from red areas is larger than distance from blue areas, it must be blue, default is red
	}

	double saturation = Optic.get_saturation(); // these probably arent necessary
	double brightness = Optic.get_brightness(); // these probably arent necessary

	// distance sensor
	int ringIntakeDist = ring_distance_sensor.get();
	// int ringPresenceConfidence = Dist1.get_confidence();

	return (ringIntakeDist <= 20 && (teamColour != redOrBlu));
	// returns true if the ring is the wrong colour
}
*/