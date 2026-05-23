#include "main.h"
#include "Subsystems/modular_lift.h"
#include "chassis.h"
#include "pros/imu.hpp"
#include "distanceReset.h"




pros::MotorGroup lift_motors({18, 19}, pros::MotorGear::blue); 
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor frontl(2, pros::MotorGear::blue);
pros::Motor frontr(-3, pros::MotorGear::blue);
pros::Motor backl(1, pros::MotorGear::blue);
pros::Motor backr(-4, pros::MotorGear::blue);
pros::Imu imu(20);

/*
pros::Distance front(8);
pros::Distance right(9);
pros::Distance left(10);
pros::Distance back(11);

DistanceSensor front_sensor(&front, 0, 0, 0);
DistanceSensor right_sensor(&right, 0, 0, 0);
DistanceSensor left_sensor(&left, 0, 0, 0);
DistanceSensor back_sensor(&back, 0, 0, 0);
*/
Chassis chassis(frontl, frontr, backl, backr, imu, {.trackWidth = 18, .wheelDiameter = 4.2, .gearRatio = 1, .kfEnabled = true});
//DistanceReset distanceReset(&chassis, {front_sensor, right_sensor, left_sensor, back_sensor}, 20, 3);

//Automatic K matrices Calculator in python
/*
import numpy as np

dt = 0.02 

A = np.array([
    [1.0, dt ],
    [0.0, 1.0]
])

B = np.array([
    [0.0],
    [0.1]
])


Q = np.array([
    [10.0, 0.0],  # Positional weight (Higher = fight position errors harder)
    [0.0,  1.0]   # Velocity weight (Higher = resist moving too fast / dampen oscillations)
])

R = np.array([[1.0]]) 

def dlqr_numpy(A, B, Q, R):
    P = Q.copy()
    for _ in range(100):
        term1 = A.T @ P @ A
        term2 = A.T @ P @ B
        term3 = np.linalg.inv(R + B.T @ P @ B)
        term4 = B.T @ P @ A
        P_next = term1 - (term2 @ term3 @ term4) + Q
        
        if np.max(np.abs(P_next - P)) < 1e-10:
            P = P_next
            break
        P = P_next
        
    K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)
    return K

K = dlqr_numpy(A, B, Q, R)

]
print(f".K = Eigen::Matrix<float, 1, 2>{{{{{K[0][0]:.4f}f, {K[0][1]:.4f}f}}}},")
*/

LiftConfig my_lift_config = {
	.K = Eigen::Matrix<float, 1, 2>{{2.9331f, 1.4557f}},
    .kG = 1750.0f,                                  // Millivolts required to hold arm horizontal
    .gear_ratio = 12.0f / 84.0f,                    // E.g., 12T gear driving an 84T gear
    .tolerance = 5.0f                               // 5 degrees error is "close enough"
};

ModularLift my_lift(&lift_motors, LiftMechanism::CASCADE, my_lift_config);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
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
void initialize() {
	//lift_motors.tare_position();
	pros::lcd::initialize();
	chassis.setPose(0,0,0);
	chassis.setXGains({
    {36.0, {15, 0, 1.2}},   // large errors
    {0.0,  {20, 0, 1.5}},   // catch-all for small errors (lookahead range)
	});
	chassis.setYGains({
		{36.0, {15, 0, 1.2}},
		{0.0,  {20, 0, 1.5}},
	});
	chassis.setThetaGains({
		{90.0, {2.15, 0.015, 0.155}},
		{0.0,  {3.0,  0.02,  0.2}},  // catch-all
	});
	imu.reset(true);
	master.rumble(".");
	
	pros::Task screen_task([&]() {
        while (true) {
            Pose pose = chassis.getPose(false);
			pros::lcd::print(0, "X: %.2f", pose.x);
			pros::lcd::print(1, "Y: %.2f", pose.y);
			pros::lcd::print(2, "Theta: %.2f", pose.theta);
			pros::delay(50);
        }
    });
	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	chassis.setPose(0,0,0);
	my_lift.cancel();
}

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
void autonomous() {
	chassis.setPose(0,0,0);
	std::string path = R"(
	0, 0, 74.223
0.14, 1.993, 70.722
0.574, 3.941, 67.042
1.347, 5.781, 63.839
2.546, 7.375, 65.18
4.047, 8.688, 73.84
5.789, 9.663, 78.568
7.649, 10.395, 86.341
9.584, 10.894, 89.241
11.552, 11.248, 91.585
13.537, 11.489, 93.483
15.531, 11.641, 96.342
17.529, 11.728, 96.122
19.529, 11.773, 93.442
21.529, 11.791, 90.683
23.529, 11.795, 87.838
25.529, 11.798, 84.897
27.529, 11.811, 81.85
29.528, 11.849, 78.686
31.527, 11.927, 75.389
33.522, 12.067, 71.941
35.509, 12.286, 68.32
37.482, 12.611, 64.496
39.425, 13.08, 60.432
41.304, 13.762, 56.075
43.08, 14.675, 51.352
44.635, 15.926, 46.154
45.901, 17.465, 40.297
46.76, 19.266, 33.425
47.271, 21.195, 24.703
47.462, 23.59, 0
47.462, 23.59, 0

	)";
	//chassis.followPathPID(parsePathData(path, false), 4.0f, {.async = false}, Chassis::HeadingMode::HoldAngle, 0);
	chassis.moveToPoint(0, 36, {.minSpeed = 35, .exitRange = 7, .timeout = 5000, .async = true}, false);
	chassis.moveToPoint(36, 36, { .timeout = 5000, .async = true}, false);

	
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
void opcontrol() {
	DriveCurve movement_curve{.curve_multipler = 1, .deadzone = 0, .minimum_output = 0};
	DriveCurve rotation_curve{.curve_multipler = 1, .deadzone = 0, .minimum_output = 0};
	int prev_forward = 0;
	int prev_sideways = 0;
	int prev_rotation = 0;
	while (true) {

		// Arcade control scheme
		int forward = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int sideways = master.get_analog(ANALOG_LEFT_X);  // Gets the turn left/right from right joystick
		int rotation = master.get_analog(ANALOG_RIGHT_X);
		if(prev_forward != forward || prev_sideways != sideways || prev_rotation != rotation) {
			std::cout << "Forward: " << forward << " Sideways: " << sideways << " Rotation: " << rotation << std::endl;
			prev_forward = forward;
			prev_sideways = sideways;
			prev_rotation = rotation;
		}
		//std::cout << "Forward: " << forward << " Sideways: " << sideways << " Rotation: " << rotation << std::endl;
		chassis.driveControl(forward, sideways, rotation, {.movement = movement_curve, .rotation = rotation_curve});                    // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}