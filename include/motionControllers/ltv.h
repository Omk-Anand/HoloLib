#pragma once

#include "Eigen/Dense"
#include "pros/rtos.hpp"

#include <atomic>
#include <cstdint>
#include <vector>

class Chassis;
struct Path;
struct Pose;

struct State {

  float x;
  float y;

  float heading;
};

class HolonomicLQR {

public:
  struct Config {

    float q_x = 18.0f;
    float q_y = 18.0f;
    float q_theta = 10.0f;

    float r_x = 0.9f;
    float r_y = 0.9f;
    float r_theta = 0.7f;

    float max_x_voltage = 12000.0f;
    float max_y_voltage = 12000.0f;
    float max_theta_voltage = 12000.0f;

    float position_tolerance = 0.5f;
    float theta_tolerance = 0.03f;

    uint32_t timeout = 5000;

    bool log = false;
  };

  HolonomicLQR(Chassis *chassis_ptr, float dtSeconds = 0.01f);

  void followPath(const Path &path);

  void moveToPose(float tx, float ty, float targetThetaRad,
                  uint32_t timeout = 5000);

  void cancel();

  bool isRunning();

  void waitUntilDone();

private:
  Chassis *chassis;

  float dt;

  Eigen::Matrix3f K;

  std::atomic<bool> isRunningFlag{false};

  std::atomic<bool> cancelFlag{false};

  void computeLQRGain();

  Eigen::Matrix3f dareSolver(const Eigen::Matrix3f &A, const Eigen::Matrix3f &B,
                             const Eigen::Matrix3f &Q,
                             const Eigen::Matrix3f &R);

  static std::vector<State> prepareTrajectory(const Path &path);

  static float angleError(float current, float target);

  static float clamp(float value, float min, float max);
};
