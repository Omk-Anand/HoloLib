#include "main.h"
#include "Subsystems/modular_lift.h"
#include "chassis.h"
#include "distanceReset.h"
#include "pros/imu.hpp"

std::vector<LiftMotorConfig> lift_motor_configs = {{18, pros::MotorGear::blue},
                                                   {19, pros::MotorGear::blue}};

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor frontl(-3, pros::MotorGear::blue);
pros::Motor frontr(2, pros::MotorGear::blue);
pros::Motor backl(-4, pros::MotorGear::blue);
pros::Motor backr(1, pros::MotorGear::blue);
pros::Imu imu(10);

Chassis chassis(frontl, frontr, backl, backr, imu,
                {
                 .drivetrainWidth = 9.1,
                 .drivetrainLength = 10.25,
                 .wheelDiameter = 4.04,
                 .gearRatio = 0.5,
                 .kfEnabled = true});

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
  /*
    chassis.setPose(0,0,0);
    chassis.moveToPose(25, 0, 90, {.minSpeed = 65, .earlyExitRange = 7});
    chassis.moveToPose(35, 25, 180, {});
    */

  std::string path = R"(
0, 0, 84.915
0.099, 1.997, 83.594
0.389, 3.974, 82.484
0.881, 5.911, 81.675
1.58, 7.783, 81.228
2.517, 9.548, 81.487
3.647, 11.196, 82.124
4.952, 12.709, 84.053
6.424, 14.062, 85.181
8.012, 15.275, 86.325
9.703, 16.341, 88.485
11.477, 17.264, 89.447
13.307, 18.068, 90.314
15.184, 18.758, 91.76
17.101, 19.329, 92.346
19.043, 19.805, 92.847
21.005, 20.192, 93.271
22.981, 20.496, 93.623
24.969, 20.714, 93.313
26.964, 20.854, 90.55
28.962, 20.922, 87.7
30.962, 20.921, 84.755
32.961, 20.85, 81.703
34.955, 20.702, 78.533
36.943, 20.483, 75.23
38.922, 20.192, 71.774
40.886, 19.82, 68.144
42.832, 19.358, 64.309
44.754, 18.805, 60.231
46.635, 18.127, 55.856
48.463, 17.318, 51.109
49.065, 15.431, 45.923
49.52, 13.484, 40.013
49.807, 11.506, 33.067
49.731, 9.515, 24.233
48.757, 7.83, 20
46.852, 7.379, 48.246
44.887, 7.725, 77.881
42.997, 8.378, 89.682
41.159, 9.165, 94.96
39.353, 10.024, 95.053
37.569, 10.928, 92.343
35.798, 11.857, 89.55
34.033, 12.798, 86.667
32.27, 13.742, 83.685
30.505, 14.683, 80.592
28.734, 15.613, 77.377
26.954, 16.524, 74.021
25.162, 17.413, 70.506
23.356, 18.271, 66.807
21.533, 19.093, 62.89
19.689, 19.868, 58.713
17.824, 20.589, 54.215
15.934, 21.243, 49.309
14.017, 21.811, 43.857
12.07, 22.269, 37.623
10.099, 22.6, 30.129
8.108, 22.777, 20
7.352, 22.799, 0
  )";
  chassis.setPose(0,0,0);
  chassis.followPathPID(parsePathData(path), 7, {}, Chassis::HeadingMode::FollowPath, 0, false);
}

void opcontrol() {
  chassis.setPose(0, 0, 90);
  chassis.setEKFstate(true);
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
