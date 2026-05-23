#include "main.h"
#include "Subsystems/modular_lift.h"
#include "chassis.h"
#include "distanceReset.h"
#include "pros/imu.hpp"

pros::MotorGroup lift_motors({18, 19}, pros::MotorGear::blue);
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor frontl(2, pros::MotorGear::blue);
pros::Motor frontr(-3, pros::MotorGear::blue);
pros::Motor backl(1, pros::MotorGear::blue);
pros::Motor backr(-4, pros::MotorGear::blue);
pros::Imu imu(20);

Chassis chassis(frontl, frontr, backl, backr, imu,
                {.trackWidth = 18,
                 .wheelDiameter = 4.2,
                 .gearRatio = 1,
                 .kfEnabled = true});

LiftConfig my_lift_config = {.K =
                                 Eigen::Matrix<float, 1, 2>{{2.9331f, 1.4557f}},
                             .kG = 1750.0f,
                             .gear_ratio = 12.0f / 84.0f,
                             .tolerance = 5.0f};

ModularLift my_lift(&lift_motors, LiftMechanism::CASCADE, my_lift_config);

void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

void initialize() {

  pros::lcd::initialize();
  chassis.setPose(0, 0, 0);
  chassis.setXGains({
      {36.0, {15, 0, 2.4}},
      {0.0, {20, 0, 3.5}},
  });
  chassis.setYGains({
      {36.0, {15, 0, 1.2}},
      {0.0, {20, 0, 1.5}},
  });
  chassis.setThetaGains({
      {90.0, {2.15, 0.015, 0.155}},
      {0.0, {3.0, 0.02, 0.2}},
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

void disabled() {
  chassis.setPose(0, 0, 0);
  my_lift.cancel();
}

void competition_initialize() {}

void autonomous() {
  chassis.setPose(0, 0, 0);
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
}

void opcontrol() {
  DriveCurve movement_curve{
      .curve_multipler = 1, .deadzone = 0, .minimum_output = 0};
  DriveCurve rotation_curve{
      .curve_multipler = 1, .deadzone = 0, .minimum_output = 0};
  int prev_forward = 0;
  int prev_sideways = 0;
  int prev_rotation = 0;
  while (true) {

    int forward = master.get_analog(ANALOG_LEFT_Y);
    int sideways = master.get_analog(ANALOG_LEFT_X);
    int rotation = master.get_analog(ANALOG_RIGHT_X);
    if (prev_forward != forward || prev_sideways != sideways ||
        prev_rotation != rotation) {
      std::cout << "Forward: " << forward << " Sideways: " << sideways
                << " Rotation: " << rotation << std::endl;
      prev_forward = forward;
      prev_sideways = sideways;
      prev_rotation = rotation;
    }

    chassis.driveControl(
        forward, sideways, rotation,
        {.movement = movement_curve, .rotation = rotation_curve});
    pros::delay(20);
  }
}
