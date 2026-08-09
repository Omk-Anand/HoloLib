#include "main.h"
#include "Eigen/Core" // IWYU pragma: export
#include "hololib/chassis.hpp"
#include "hololib/config.hpp"
#include "hololib/localization/ApriltagLocalization.hpp"
#include "hololib/localization/odometry.hpp"
#include "hololib/motions/motion_handler.hpp"
#include "hololib/motions/motions.hpp"
#include "hololib/util/GainScheduler.hpp"
#include "hololib/util/modular_lift.hpp"
#include "hololib/util/replay.hpp"
#include "pros/ai_vision.hpp"
#include "pros/imu.hpp"

#include <cstdio>
#include <print>

// Motor ports (negative for reversing motor)
int frontLPort = 3;
int frontRPort = 2;
int backLPort = 4;
int backRPort = 1;

// IMU port
int imuPort = 10;

std::vector<LiftMotorConfig> lift_motor_configs = {
    {18, pros::MotorGear::blue},
    {19, pros::MotorGear::blue}
};

Eigen::Matrix3f cameraMatrix = (Eigen::Matrix3f() << 383.57019326565296f * 0.5f,
                                0.0f,
                                322.50382974986405f * 0.5f,
                                0.0f,
                                385.93627187500545f * 0.5f,
                                230.00229561364546f * 0.5f,
                                0.0f,
                                0.0f,
                                1.0f)
                                   .finished();

Eigen::Vector<float, 5> distCoeffs = (Eigen::Vector<float, 5>() << -0.047271311979687029f,
                                      0.19589273312693006f,
                                      -0.0052232338824990303f,
                                      0.00058677141664816013f,
                                      -0.14778998923627218f)
                                         .finished();

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Initalize motors, IMU, odometry, and chassis
pros::Motor frontLeft = pros::Motor(frontLPort, pros::MotorGear::blue);
pros::Motor frontRight = pros::Motor(frontRPort, pros::MotorGear::blue);
pros::Motor backLeft = pros::Motor(backLPort, pros::MotorGear::blue);
pros::Motor backRight = pros::Motor(backRPort, pros::MotorGear::blue);
pros::Imu imu = pros::Imu(imuPort);
pros::AIVision visionSensor = pros::AIVision(8);

// Initialize odometry configuration. This has to be constructed before chassis:
// EncoderEKFOdometry reads the motors and IMU in its constructor, and chassis
// binds a reference to it, so defining chassis first would capture an object
// that has not run its constructor yet.
hololib::ChassisConfig chassis_config = {
    .drivetrainWidth = 9.1, .drivetrainLength = 10.25, .wheelDiameter = 3.25, .gearRatio = 0.5};
hololib::EncoderEKFOdometry odom =
    hololib::EncoderEKFOdometry(frontLeft, frontRight, backLeft, backRight, imu, chassis_config);
const std::function<hololib::Pose(bool)> poseGetter = [](bool radians) { return odom.getPose(radians); };

hololib::Chassis chassis = hololib::Chassis(frontLeft, frontRight, backLeft, backRight, imu, odom);
hololib::GainScheduler xSched = hololib::GainScheduler();
hololib::GainScheduler ySched = hololib::GainScheduler();
hololib::GainScheduler thetaSched = hololib::GainScheduler();
hololib::ApriltagLocalization tagOdom =
    hololib::ApriltagLocalization(visionSensor, imu, poseGetter, cameraMatrix, distCoeffs);

// Initialize obstacle manager
hololib::ObstacleManager obstacles = hololib::ObstacleManager();

// Initialize lift motor configs
LiftConfig my_lift_config = {
    .gear_ratio = 12.0f / 84.0f,
    .arm_length = 15.0f,
    .arm_mass_kg = 2.0f,
    .payload_mass_kg = 0.0f,
    .kG_base = 1750.0f / (2.0f * 9.81f),
    .tolerance = 5.0f,
    .K = Eigen::Matrix<float, 1, 2>{2.9331f, 1.4557f}, // Initialize k gain matrix for lqr
    .spool_radius = 1.5f
};

// Initialize lift using lqr control
ModularLift my_lift(lift_motor_configs, LiftMechanism::CASCADE, my_lift_config);

void initialize() {
    pros::lcd::initialize();

    // Calibrate the chassis
    chassis.calibrate();
    odom.startTask();
    tagOdom.startTask(100);

    // Set PID gains for chassis
    xSched.setGains({
        {36.0, {15, 0, 2.4}},
        {0.0,  {25, 0, 0.5}},
    });
    ySched.setGains({
        {36.0, {15, 0, 1.6}},
        {0.0,  {20, 0, 1.5}},
    });
    thetaSched.setGains({
        {90.0, {2.76411f, 0.0116046f, 0.0384008f}},
        {0,    {3, 0, 0.04}                      }
    });

    // Basically allows you to see the velocity of the chassis (in/s) (helpful
    // for making custom motions)
    odom.setVelocityCalculations(true);

    // LCD screen task to display chassis data
    pros::Task screen_task([&]() {
        while (true) {
            hololib::Pose pose = odom.getPose(false); // false means degrees, true means radians
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
    odom.setPose(0, 0, 0);
    my_lift.cancel();
}

void competition_initialize() {}

/*
Run:
python tools/sim_auton.py
then open the file with the browser of your choice.
*/
void simulation() {}

void autonomous() {
    // Example autonomous routine
    odom.setPose(48, 10, 0);
    hololib::Pose p = tagOdom.getPoseFromTag(false);

    chassisAsync(hololib::moveToPoint(24, 24));
    chassisAsync(hololib::turnToPoint(12, 12));

    // Motion Chaining
    chassisAsync(hololib::moveToPoint(12, 12, {.minSpeed = 35, .earlyExitRange = 7, .timeout = 2000}));
    chassisAsync(hololib::moveToPose(0, 0, 180));

    // Relative Motions
    chassisAsync(hololib::moveRelative(20, 10));
    chassisAsync(hololib::strafeDistance(10));

    // Driver replay, motion cancelling and other features can be learned from the docs

    // Obstacle Avoidance Example
    obstacles.addObstacle(0, 0, 5); // initializes an obstacle at 0,0 with a radius of 5in

    obstacles.setAvoidanceParams(2.0f, 4.0f); // set the avoidance parameters
    obstacles.setPotentialFieldParams(1.0f, 100.0f,
                                      24.0f);                                // set the potential field parameters
    obstacles.setRobotDimensions(9.1f, 10.25f);                              // set the robot dimensions
    obstacles.setAvoidanceMode(hololib::ObstacleManager::AvoidanceMode::On); // set the avoidance mode
}

void opcontrol() {
    odom.setPose(0, 0, 0);

    // Example drive curves
    hololib::Chassis::DriveCurve movement_curve{.curve_multipler = 1.01, .deadzone = 5, .minimum_output = 5};
    hololib::Chassis::DriveCurve rotation_curve{.curve_multipler = 1.028, .deadzone = 5, .minimum_output = 5};

    // hololib::DriverReplay::logReplayData(master, 100, poseGetter); // allows logging for driver replay

    while (true) {
        // Receive inputs from controller
        int forward = master.get_analog(ANALOG_LEFT_Y);
        int sideways = master.get_analog(ANALOG_LEFT_X);
        int rotation = master.get_analog(ANALOG_RIGHT_X);

        // Detects collisions utilizing multiple information from the motors (velocity current, motor load and
        // effeciency)
        if (chassis.detectCollision()) {
            hololib::DriverReplay::getControllerInput(master); // Log controller input
            std::println("Collision Detected!");
        }

        // minor heading correction to assist drivers (only on when no joystick output)
        chassis.driveControl(forward, sideways, rotation, {.movement = movement_curve, .rotation = rotation_curve},
                             true, 0.0f, {.correctionOn = true, .kP = 0.15f, .kI = 0.01f, .kD = 0.01f});

        pros::delay(20); // delay for buffer
    }
}
