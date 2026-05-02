#pragma once
#include <vector>
#include <cmath>
#include <iostream>
#include <memory>
#include "pros/motor_group.hpp"
#include "motionControllers/velocityController.h"

// Define core tracking structs globally
struct Pose {
    float x, y, theta;
    float distance(const Pose& other) const {
        return std::hypot(x - other.x, y - other.y);
    }
};

struct Path {
    std::vector<std::pair<float, float>> Poses;
    std::vector<std::pair<float, float>> Velocities;
    int timestep_duration_msec = 10;
};

class Chassis; // Forward declare

#include "motionControllers/ltv.h" 

class Chassis {
private:
    const float INCH_TO_METER = 0.0254f;
    Pose pose{0,0,0};

    std::vector<std::int8_t> rightMotorPorts;
    std::vector<std::int8_t> leftMotorPorts;
    
    pros::MotorGroup rightMotors;
    pros::MotorGroup leftMotors;

    enum chassisModel { Differential, XDrive };
    chassisModel model = Differential;

    // Drivetrain Physical Constants
    float trackWidth = 10 * 0.0254f;
    float wheelDiameter = 3.25f; // inches
    float gearRatio = 1.0f; // Output RPM / Motor RPM
    float rpmToMpsConversion = 0.00324173f; // Default fallback

    // Velocity Controller
    float integralWindup = 5000;
    VelocityControllerConfig config{0,0,0,0,0,0,0};
    VoltageController controller;

    // LTV Control
    LTVPathFollower* ltv;
    LTVPathFollower::ltvConfig defaultLtvConfig; // Stores the global LTV settings

public:
    bool abortAuton = false; 

    Chassis(std::vector<std::int8_t> rightMotors, std::vector<std::int8_t> leftMotors);
    ~Chassis();

    void brake();
    void cancelMotion();
    
    void setChassisModelDifferential();
    void setChassisModelXDrive();
    
    void setPose(float x, float y, float theta);
    Pose getPose();
    
    bool detectCollision();
    
    void tank(int right, int left);
    void tank(float lin_vel, float ang_vel, const VelocityControllerConfig &config, unsigned int time);
    
    // --- Modular Setup API ---
    void setDrivetrainConstants(float wheel_diameter_inches, float gear_ratio, float track_width_inches);
    void setVelocityController(float kV = 0, float KA_turn = 0, float KA_straight = 0, float KS_turn = 0, 
                               float KS_straight = 0, float KP_straight = 0, float KI_straight = 0, 
                               float integral_windup = 0);
    void setLTVParameters(float q_x, float q_y, float q_theta, float r_vel, float r_ang, 
                          float max_lin_corr = 99999.0f, float max_ang_corr = 999999.0f);
    void setLTVBackwardsParameters(float q_x_b, float q_y_b, float q_theta_b, float r_vel_b, float r_ang_b);

    // --- LTV Integration API ---
    void executeVelocityCommand(float v_cmd, float w_cmd);
    
    // Overloaded to allow using default params OR specific params
    void followPath(const Path& path); 
    void followPath(const Path& path, const LTVPathFollower::ltvConfig& l_config);
    
    void waitUntilDone();
    void waitUntil(float dist_inches);
    void waitUntil(float x_inch, float y_inch, float radius_inch = 2.0f);
};