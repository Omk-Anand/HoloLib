#pragma once

#include "main.h"
#include "Eigen/Dense"

// Define the available physical models
enum class LiftMechanism {
    CASCADE,   // Linear upward motion (constant gravity)
    FOUR_BAR,  // Rotational (gravity changes with cos(theta))
    SIX_BAR,   // Similar physics to 4-bar
    VIRTUAL    // Chain-driven virtual 4-bar
};

struct LiftConfig {
    Eigen::Matrix<float, 1, 2> K; // Pre-computed LQR feedback gains [k_pos, k_vel]
    float kG;                     // Gravity feedforward constant (voltage)
    float gear_ratio;             // Gear ratio (Output / Input) e.g., 12.0/60.0
    float tolerance;              // Acceptable error in degrees to consider "done"
};

class ModularLift {
private:
    pros::MotorGroup* motors;
    LiftMechanism type;
    LiftConfig config;

    // Task management
    pros::Task* task = nullptr;
    bool is_running = false;
    bool cancel_request = false;
    
    // Control variables
    float target_position = 0.0f;
    pros::Mutex target_mutex; // Protects target_position across threads

    // Internal helpers
    float getLiftRadians(float motor_degrees);
    float calculateFeedforward(float current_motor_degrees);
    
    // Task trampoline and implementation
    struct TaskParams {
        ModularLift* instance;
    };
    static void task_trampoline(void* params);
    void controlLoopImpl();

public:
    ModularLift(pros::MotorGroup* m_group, LiftMechanism t, LiftConfig c);
    ~ModularLift();

    // Command the lift
    void moveTo(float target_motor_degrees);
    
    // State management
    void cancel();
    void waitUntilDone();
    bool isRunning();
};