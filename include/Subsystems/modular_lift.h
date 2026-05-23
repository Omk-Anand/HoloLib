#pragma once

#include "Eigen/Dense"
#include "main.h"

enum class LiftMechanism { CASCADE, FOUR_BAR, SIX_BAR, VIRTUAL };

struct LiftConfig {
  Eigen::Matrix<float, 1, 2> K;
  float kG;
  float gear_ratio;
  float tolerance;
};

class ModularLift {
private:
  pros::MotorGroup *motors;
  LiftMechanism type;
  LiftConfig config;

  pros::Task *task = nullptr;
  bool is_running = false;
  bool cancel_request = false;

  float target_position = 0.0f;
  pros::Mutex target_mutex;

  float getLiftRadians(float motor_degrees);
  float calculateFeedforward(float current_motor_degrees);

  struct TaskParams {
    ModularLift *instance;
  };
  static void task_trampoline(void *params);
  void controlLoopImpl();

public:
  ModularLift(pros::MotorGroup *m_group, LiftMechanism t, LiftConfig c);
  ~ModularLift();

  void moveTo(float target_motor_degrees);

  void cancel();
  void waitUntilDone();
  bool isRunning();
};
