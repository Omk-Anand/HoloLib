#pragma once

#include "Eigen/Core"
#include "api.h"
#include <atomic>


class ModularLift;



 
struct TaskParams {
  ModularLift* instance;
};

struct LiftMotorConfig {
  int port;
  pros::MotorGear gearset;
};



 
enum class LiftMechanism {
  CASCADE,
  FOUR_BAR,
  SIX_BAR,
  VIRTUAL
};



 
struct LiftConfig {
  float gear_ratio;
  float arm_length;
  float arm_mass_kg;
  float payload_mass_kg;
  float kG_base;
  float tolerance;
  Eigen::Matrix<float, 1, 2> K;
  float spool_radius = 1.0f;
};

class ModularLift {
public:
  





 
  ModularLift(const std::vector<LiftMotorConfig>& m_configs, LiftMechanism t, LiftConfig c);

  

 
  ~ModularLift();

  

 
  float getLiftRadians(float motor_degrees);

  

 
  void setPayload(float mass_kg);

  

 
  float calculateFeedforward(float current_motor_degrees);

  

 
  void moveTo(float target_motor_degrees);

  

 
  void cancel();

  

 
  void waitUntilDone();

  

 
  bool isRunning();

private:
  std::vector<pros::Motor> motors;
  LiftMechanism type;
  LiftConfig config;


  std::atomic<bool> is_running;
  std::atomic<bool> cancel_request;
  std::atomic<bool> is_settled;

  pros::Task *task;
  pros::Mutex target_mutex;
  float target_position = 0.0f;

  

 
  static void task_trampoline(void *params);

  

 
  void controlLoopImpl();
};