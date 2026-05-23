#include "motionControllers/ltv.h"
#include "chassis.h"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

HolonomicLQR::HolonomicLQR(Chassis *chassis_ptr, float dtSeconds)
    : chassis(chassis_ptr), dt(dtSeconds) {
  computeLQRGain();
}

float HolonomicLQR::angleError(float current, float target) {
  return std::remainder(target - current, 2.0f * M_PI);
}

float HolonomicLQR::clamp(float value, float min, float max) {
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

void HolonomicLQR::computeLQRGain() {

  Eigen::Matrix3f A = Eigen::Matrix3f::Identity();

  Eigen::Matrix3f B = dt * Eigen::Matrix3f::Identity();

  Eigen::Matrix3f Q;
  Q << 18.0f, 0, 0, 0, 18.0f, 0, 0, 0, 10.0f;

  Eigen::Matrix3f R;
  R << 0.9f, 0, 0, 0, 0.9f, 0, 0, 0, 0.7f;

  Eigen::Matrix3f P = dareSolver(A, B, Q, R);

  K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;

  std::cout << "[HLQR] Gain Matrix:\n" << K << std::endl;
}

Eigen::Matrix3f HolonomicLQR::dareSolver(const Eigen::Matrix3f &A,
                                         const Eigen::Matrix3f &B,
                                         const Eigen::Matrix3f &Q,
                                         const Eigen::Matrix3f &R) {

  Eigen::Matrix3f P = Q;

  for (int i = 0; i < 200; i++) {

    Eigen::Matrix3f P_next = A.transpose() * P * A -
                             A.transpose() * P * B *
                                 (R + B.transpose() * P * B).inverse() *
                                 B.transpose() * P * A +
                             Q;

    if ((P_next - P).norm() < 1e-4f) {
      P = P_next;
      break;
    }

    P = P_next;
  }

  return P;
}

std::vector<State> HolonomicLQR::prepareTrajectory(const Path &path) {

  size_t n = std::min(path.Poses.size(), path.Velocities.size());

  if (n == 0)
    return {};

  std::vector<State> states(n);

  for (size_t i = 0; i < n; i++) {

    states[i].x = path.Poses[i].first;

    states[i].y = path.Poses[i].second;

    if (i < n - 1) {

      float dx = path.Poses[i + 1].first - path.Poses[i].first;

      float dy = path.Poses[i + 1].second - path.Poses[i].second;

      states[i].heading = std::atan2(dy, dx);

    } else if (i > 0) {

      states[i].heading = states[i - 1].heading;

    } else {

      states[i].heading = 0;
    }
  }

  return states;
}

void HolonomicLQR::followPath(const Path &path) {

  if (isRunningFlag) {
    cancel();
    waitUntilDone();
  }

  isRunningFlag = true;
  cancelFlag = false;

  std::vector<State> trajectory = prepareTrajectory(path);

  if (trajectory.empty()) {

    std::cout << "[HLQR] Empty trajectory." << std::endl;

    isRunningFlag = false;
    return;
  }

  uint32_t now = pros::millis();

  for (size_t i = 0; i < trajectory.size(); i++) {

    if (cancelFlag)
      break;

    const State &target = trajectory[i];

    Pose current = chassis->getPose(true);

    Eigen::Vector3f globalError;

    globalError <<

        target.x - current.x,

        target.y - current.y,

        angleError(current.theta, target.heading);

    float theta = current.theta;

    Eigen::Matrix3f rotation;

    rotation <<

        std::cos(theta),
        std::sin(theta), 0,

        -std::sin(theta), std::cos(theta), 0,

        0, 0, 1;

    Eigen::Vector3f localError = rotation * globalError;

    Eigen::Vector3f u = K * localError;

    float vx = clamp(u(0), -12000, 12000);

    float vy = clamp(u(1), -12000, 12000);

    float vt = clamp(u(2), -12000, 12000);

    chassis->setMotorVoltages(chassis->calculateHolonomic(vx, vy, vt));

    pros::Task::delay_until(&now, path.timestep_duration_msec);
  }

  chassis->brake();

  isRunningFlag = false;
}

void HolonomicLQR::moveToPose(float tx, float ty, float targetThetaRad,
                              uint32_t timeout) {

  uint32_t start = pros::millis();

  while (pros::millis() - start < timeout) {

    Pose current = chassis->getPose(true);

    Eigen::Vector3f globalError;

    globalError <<

        tx - current.x,

        ty - current.y,

        angleError(current.theta, targetThetaRad);

    float distanceError = std::hypot(globalError(0), globalError(1));

    if (distanceError < 0.5f && std::abs(globalError(2)) < 0.03f) {
      break;
    }

    float theta = current.theta;

    Eigen::Matrix3f rotation;

    rotation <<

        std::cos(theta),
        std::sin(theta), 0,

        -std::sin(theta), std::cos(theta), 0,

        0, 0, 1;

    Eigen::Vector3f localError = rotation * globalError;

    Eigen::Vector3f u = K * localError;

    float vx = clamp(u(0), -12000, 12000);

    float vy = clamp(u(1), -12000, 12000);

    float vt = clamp(u(2), -12000, 12000);

    chassis->setMotorVoltages(chassis->calculateHolonomic(vx, vy, vt));

    pros::delay(10);
  }

  chassis->brake();
}

void HolonomicLQR::cancel() { cancelFlag = true; }

bool HolonomicLQR::isRunning() { return isRunningFlag; }

void HolonomicLQR::waitUntilDone() {

  while (isRunningFlag) {
    pros::delay(10);
  }
}
