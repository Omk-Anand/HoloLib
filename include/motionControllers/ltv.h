#pragma once

#include "Eigen/Dense"
#include "pros/rtos.hpp"

#include <vector>
#include <atomic>
#include <cstdint>

// ============================================================
// FORWARD DECLARATIONS
// ============================================================

class Chassis;
struct Path;
struct Pose;

// ============================================================
// TRAJECTORY STATE
// ============================================================

struct State {

    float x;
    float y;

    // Radians
    float heading;
};

// ============================================================
// HOLOMONIC LQR CONTROLLER
// ============================================================

class HolonomicLQR {

public:

    // ========================================================
    // CONFIG
    // ========================================================

    struct Config {

        // ================================================
        // STATE COST MATRIX (Q)
        // ================================================

        float q_x = 18.0f;
        float q_y = 18.0f;
        float q_theta = 10.0f;

        // ================================================
        // CONTROL COST MATRIX (R)
        // ================================================

        float r_x = 0.9f;
        float r_y = 0.9f;
        float r_theta = 0.7f;

        // ================================================
        // MAX VOLTAGES
        // ================================================

        float max_x_voltage = 12000.0f;
        float max_y_voltage = 12000.0f;
        float max_theta_voltage = 12000.0f;

        // ================================================
        // EXIT CONDITIONS
        // ================================================

        float position_tolerance = 0.5f;
        float theta_tolerance = 0.03f;

        // ================================================
        // TIMING
        // ================================================

        uint32_t timeout = 5000;

        // ================================================
        // DEBUGGING
        // ================================================

        bool log = false;
    };

    // ========================================================
    // CONSTRUCTOR
    // ========================================================

    HolonomicLQR(
        Chassis* chassis_ptr,
        float dtSeconds = 0.01f
    );

    // ========================================================
    // PATH FOLLOWING
    // ========================================================

    void followPath(
        const Path& path
    );

    // ========================================================
    // POINT-TO-POINT
    // ========================================================

    void moveToPose(
        float tx,
        float ty,
        float targetThetaRad,
        uint32_t timeout = 5000
    );

    // ========================================================
    // TASK CONTROL
    // ========================================================

    void cancel();

    bool isRunning();

    void waitUntilDone();

private:

    // ========================================================
    // INTERNALS
    // ========================================================

    Chassis* chassis;

    float dt;

    Eigen::Matrix3f K;

    std::atomic<bool> isRunningFlag {false};

    std::atomic<bool> cancelFlag {false};

    // ========================================================
    // LQR
    // ========================================================

    void computeLQRGain();

    Eigen::Matrix3f dareSolver(
        const Eigen::Matrix3f& A,
        const Eigen::Matrix3f& B,
        const Eigen::Matrix3f& Q,
        const Eigen::Matrix3f& R
    );

    // ========================================================
    // TRAJECTORY
    // ========================================================

    static std::vector<State> prepareTrajectory(
        const Path& path
    );

    // ========================================================
    // HELPERS
    // ========================================================

    static float angleError(
        float current,
        float target
    );

    static float clamp(
        float value,
        float min,
        float max
    );
};