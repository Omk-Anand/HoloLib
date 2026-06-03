#include "motionControllers/ltv.h"
#include "chassis.h"
#include <cmath>
#include <iostream>

HolonomicLQR::HolonomicLQR(Chassis *chassis_ptr, const Config &cfg,
                           float dtSeconds)
    : chassis(chassis_ptr), config(cfg), dt(dtSeconds) {
  computeLQRGain();
}

HolonomicLQR::HolonomicLQR(Chassis *chassis_ptr, float dtSeconds)
    : chassis(chassis_ptr), dt(dtSeconds) {
  computeLQRGain();
}

void HolonomicLQR::computeLQRGain() {

  Eigen::Matrix3f A = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f B = Eigen::Matrix3f::Identity() * dt;

  Eigen::Matrix3f Q = Eigen::Matrix3f::Zero();
  Q(0, 0) = config.q_x;
  Q(1, 1) = config.q_y;
  Q(2, 2) = config.q_theta;

  Eigen::Matrix3f R = Eigen::Matrix3f::Zero();
  R(0, 0) = config.r_x;
  R(1, 1) = config.r_y;
  R(2, 2) = config.r_theta;

  Eigen::Matrix3f P = dareSolver(A, B, Q, R);
  Eigen::Matrix3f temp = R + B.transpose() * P * B;
  K = temp.inverse() * B.transpose() * P * A;
}

Eigen::Matrix3f HolonomicLQR::dareSolver(const Eigen::Matrix3f &A,
                                         const Eigen::Matrix3f &B,
                                         const Eigen::Matrix3f &Q,
                                         const Eigen::Matrix3f &R) {
  Eigen::Matrix3f P = Q;
  const int max_iter = 500;
  const float tolerance = 1e-5;

  for (int i = 0; i < max_iter; ++i) {
    Eigen::Matrix3f temp = R + B.transpose() * P * B;
    Eigen::Matrix3f P_next =
        A.transpose() * P * A -
        A.transpose() * P * B * temp.inverse() * B.transpose() * P * A + Q;

    if ((P_next - P).norm() < tolerance) {
      P = P_next;
      break;
    }
    P = P_next;
  }
  return P;
}

std::vector<State> HolonomicLQR::prepareTrajectory(const Path &path) {
  std::vector<State> traj;
  for (const auto &pt : path) {
    traj.push_back({pt.x, pt.y, pt.theta});
  }
  return traj;
}

float HolonomicLQR::angleError(float current, float target) {
  return std::remainder(target - current, 360.0f);
}

float HolonomicLQR::clamp(float value, float min, float max) {
  return std::max(min, std::min(value, max));
}

void HolonomicLQR::cancel() {
  cancelFlag = true;
  chassis->brake();
}

bool HolonomicLQR::isRunning() { return isRunningFlag; }

void HolonomicLQR::waitUntilDone() {
  while (isRunningFlag) {
    pros::delay(10);
  }
}

void HolonomicLQR::followPath(const Path &path) {
  if (path.empty())
    return;

  chassis->motion.enqueue(
      [=, this]() {
        isRunningFlag = true;
        cancelFlag = false;

        auto traj = prepareTrajectory(path);
        uint32_t start_time = pros::millis();
        uint32_t settle_start = 0;

        int closest_idx = 0;

        while (!cancelFlag && (pros::millis() - start_time < config.timeout)) {
          Pose curr = chassis->getPose(false);

          float best_dist = std::numeric_limits<float>::max();
          for (int i = closest_idx; i < traj.size(); ++i) {
            float d = std::hypot(traj[i].x - curr.x, traj[i].y - curr.y);
            if (d < best_dist) {
              best_dist = d;
              closest_idx = i;
            }
          }

          int ref_idx = std::min((int)traj.size() - 1, closest_idx + 2);
          State ref = traj[ref_idx];

          float ex = ref.x - curr.x;
          float ey = ref.y - curr.y;
          float etheta = angleError(curr.theta, ref.heading);


          if (ref_idx == traj.size() - 1) {
            if (std::hypot(ex, ey) < config.position_tolerance &&
                std::abs(etheta) < config.theta_tolerance) {
              if (settle_start == 0)
                settle_start = pros::millis();
              if (pros::millis() - settle_start > 150)
                break;
            } else {
              settle_start = 0;
            }
          }


          float rad = curr.theta * M_PI / 180.0f;
          float cos_t = std::cos(rad);
          float sin_t = std::sin(rad);

          float ex_local = ex * sin_t + ey * cos_t;
          float ey_local = ex * cos_t - ey * sin_t;

          Eigen::Vector3f error_state(ey_local, ex_local,
                                      etheta);


          Eigen::Vector3f u = K * error_state;

          float vx = clamp(u(0), -config.max_x_voltage, config.max_x_voltage);
          float vy = clamp(u(1), -config.max_y_voltage, config.max_y_voltage);
          float w =
              clamp(u(2), -config.max_theta_voltage, config.max_theta_voltage);

          chassis->setMotorVoltages(chassis->calculateHolonomic(vx, vy, w));
          pros::delay(dt * 1000);
        }

        chassis->brake();
        isRunningFlag = false;
      },
      true);
}

void HolonomicLQR::moveToPose(float tx, float ty, float targetThetaRad,
                              uint32_t timeout) {
  Path p = {{tx, ty, targetThetaRad * 180.0f / (float)M_PI}};
  uint32_t old_timeout = config.timeout;
  config.timeout = timeout;
  followPath(p);
  config.timeout = old_timeout;
}
