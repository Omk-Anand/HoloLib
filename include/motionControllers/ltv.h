#pragma once

#include "Eigen/Dense"
#include "pros/rtos.hpp"
#include <vector>
#include <string>
#include <atomic>

// Forward declarations
class Chassis;
struct Path;
struct Pose;

struct State {
    double x;
    double y;
    double heading;
    double linear_vel;
    double angular_vel;
};

class LTVPathFollower {
public:
    struct ltvConfig {
        bool backwards = false;
        bool log = false;
        bool test = false;
        bool turnFirst = false;
        float max_lin_correction = 99999.0f;
        float max_ang_correction = 999999.0f;

        float q_x = 10;
        float q_y = 270.0;
        float q_theta = 0.25f * 30;
        float r_ang = 0.25f;
        float r_vel = 1.0;

        float q_x_b = 5;
        float q_y_b = 270;
        float q_theta_b = 0.01;
        float r_ang_b = 0.3; 
        float r_vel_b = 1.0f;
        
        float q_scalar = 1.0f; 
    };

    LTVPathFollower(Chassis* chassis_ptr);
    
    void followPath(const Path& path, const ltvConfig& l_config);
    void waitUntilDone();
    void waitUntil(float dist_inches);
    void waitUntil(float x_inch, float y_inch, float radius_inch = 2.0f);
    void cancel();
    bool isRunning();

private:
    static constexpr float INCH_TO_METER = 0.0254f;

    Chassis* chassis; // Reference back to control chassis properties

    pros::Task* task = nullptr;
    std::atomic<bool> is_running {false};
    std::atomic<bool> cancel_request {false};
    std::atomic<float> distance_traveled_inches {0.0f};
    
    struct TaskParams {
        LTVPathFollower* instance;
        Path path;
        ltvConfig config;
    };

    static void task_trampoline(void* params);
    void followPathImpl(const Path& path, const ltvConfig& l_config);
    
    Eigen::MatrixXf dareSolver(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R);
    std::pair<Eigen::MatrixXf, Eigen::MatrixXf> discretizeAB(const Eigen::MatrixXf& contA, const Eigen::MatrixXf& contB, double dtSeconds);
    
    static std::vector<State> prepare_trajectory(const Path& path);
    static double angleError(double robotAngle, double targetAngle);
    static double clamp(double value, double min, double max);

    class Vector2 {
    public:
        Vector2(float x, float y);
        std::string latex() const;
        float x, y;
    };
};