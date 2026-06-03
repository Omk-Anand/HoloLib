#include "main.h"
#include "Subsystems/modular_lift.h"
#include "chassis.h"
#include "distanceReset.h"
#include "pros/imu.hpp"

std::vector<LiftMotorConfig> lift_motor_configs = {{18, pros::MotorGear::blue},
                                                   {19, pros::MotorGear::blue}};

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor frontl(2, pros::MotorGear::blue);
pros::Motor frontr(-3, pros::MotorGear::blue);
pros::Motor backl(1, pros::MotorGear::blue);
pros::Motor backr(-4, pros::MotorGear::blue);
pros::Imu imu(20);

Chassis chassis(frontl, frontr, backl, backr, imu,
                {.trackWidth = 13,
                 .drivetrainWidth = 12,
                 .drivetrainLength = 13.25,
                 .wheelDiameter = 4.05,
                 .gearRatio = 1,
                 .kfEnabled = false});

LiftConfig my_lift_config = {.gear_ratio = 12.0f / 84.0f,
                             .arm_length = 15.0f,
                             .arm_mass_kg = 2.0f,
                             .payload_mass_kg = 0.0f,
                             .kG_base = 1750.0f / (2.0f * 9.81f),
                             .tolerance = 5.0f,
                             .K =
                                 Eigen::Matrix<float, 1, 2>{{2.9331f, 1.4557f}},
                             .spool_radius = 1.5f};

ModularLift my_lift(lift_motor_configs, LiftMechanism::CASCADE, my_lift_config);

void initialize() {

  pros::lcd::initialize();
  chassis.calibrate();
  chassis.setPose(0, 0, 0);
  chassis.setXGains({
      {36.0, {15, 0, 2.4}},
      {0.0, {20, 0, 1}},
  });
  chassis.setYGains({
      {36.0, {15, 0, 1.6}},
      {0.0, {20, 0, 1.5}},
  });
  chassis.setThetaGains({
      {90.0, {2.15, 0.015, 0.12}},
      {0.0, {3.0, 0.02, 0.1}},
  });
  chassis.setVelocityCalculations(true);
  pros::Task screen_task([&]() {
    while (true) {
      Pose pose = chassis.getPose(false);
      pros::lcd::print(0, "X: %.3f", pose.x);
      pros::lcd::print(1, "Y: %.3f", pose.y);
      pros::lcd::print(2, "Theta: %.3f", pose.theta);
      pros::lcd::print(3, "X Velocity: %.3f", pose.velocity.vx);
      pros::lcd::print(4, "Y Velocity: %.3f", pose.velocity.vy);
      pros::lcd::print(5, "Theta Velocity: %.3f", pose.velocity.w);
      pros::delay(50);
    }
  });
}

void disabled() {
  chassis.setPose(0, 0, 0);
  my_lift.cancel();
}

void competition_initialize() {}







 
void simulation() {
  chassis.setPose(-24, 0, 90);
  chassis.moveToPoint(-24, 24);
  chassis.moveToPose(24, 24, 180, {});
  chassis.moveToPoint(24, -24);
  chassis.turnToPoint(-24, -24);
  chassis.moveToPoint(-24, -24, {}, true);
  chassis.moveToPoint(-24, 0);
}

void autonomous() {
  






 
  std::string path = R"(39.534, 25.961, 94.718
39.49, 23.962, 94.718
39.393, 21.965, 94.108
39.233, 19.971, 93.504
38.999, 17.985, 93.504
38.668, 16.013, 92.936
38.254, 14.057, 92.438
37.752, 12.121, 92.047
37.157, 10.213, 91.795
36.465, 8.337, 91.709
35.676, 6.5, 91.801
34.792, 4.707, 92.072
33.816, 2.962, 92.511
32.755, 1.267, 93.093
31.616, -0.377, 93.791
30.409, -1.971, 94.573
29.144, -3.519, 95.41
27.829, -5.026, 96.275
26.474, -6.497, 97.149
25.089, -7.94, 98.014
23.682, -9.362, 98.858
22.262, -10.769, 99.672
20.835, -12.171, 99.549
19.409, -13.573, 98.811
17.99, -14.983, 98.12
16.586, -16.407, 97.48
15.202, -17.85, 96.897
13.843, -19.318, 95.437
12.515, -20.813, 92.738
11.222, -22.339, 89.957
9.97, -23.898, 87.088
8.761, -25.491, 84.121
7.605, -27.124, 81.045
6.503, -28.793, 77.849
5.455, -30.495, 74.515
4.46, -32.23, 71.025
3.518, -33.994, 67.355
2.632, -35.787, 63.472
1.817, -37.613, 59.335
1.054, -39.461, 54.889
0.338, -41.328, 50.049
-0.317, -43.218, 44.686
-0.911, -45.128, 38.587
-1.468, -47.048, 31.321
-1.967, -48.985, 21.749
-2.401, -50.796, 270
)";

  chassis.addObstacle(0, 0, 5.5);
  chassis.setAvoidanceMode(Chassis::AvoidanceMode::On);
  chassis.setAvoidanceParams(7, 10);
  chassis.setPose(-24, 24, 180);
  chassis.moveToPoint(24, -24, {}, true);
}

void opcontrol() {
  chassis.setPose(0, 0, 90);
  chassis.setEKFstate(false);
  DriveCurve movement_curve{
      .curve_multipler = 1.01, .deadzone = 5, .minimum_output = 5};
  DriveCurve rotation_curve{
      .curve_multipler = 1.028, .deadzone = 5, .minimum_output = 5};
  int prev_forward = 0;
  int prev_sideways = 0;
  int prev_rotation = 0;
  while (true) {

    int forward = master.get_analog(ANALOG_LEFT_Y);
    int sideways = master.get_analog(ANALOG_LEFT_X);
    int rotation = master.get_analog(ANALOG_RIGHT_X);
    if (prev_forward != forward || prev_sideways != sideways ||
        prev_rotation != rotation) {
      prev_forward = forward;
      prev_sideways = sideways;
      prev_rotation = rotation;
    }

    if (chassis.detectCollision()) {
      std::cout << "Collision Detected!" << std::endl;
    }
    chassis.driveControl(
        forward, sideways, rotation,
        {.movement = movement_curve, .rotation = rotation_curve}, true, 90,
        {.correctionOn = true, .kP = 0.15f, .kI = 0.01f, .kD = 0.01f});
    pros::delay(20);
  }
}
