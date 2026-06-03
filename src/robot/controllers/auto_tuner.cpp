#include "auto_tuner.h"
#include "chassis.h"
#include "api.h"
#include "Eigen/Dense"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>

struct SimplexVertex {
  Eigen::Vector3f point;
  float cost;
  
  bool operator<(const SimplexVertex& other) const {
    return cost < other.cost;
  }
};

float evaluateGains(Chassis* chassis, TuneTarget target, float dist, uint32_t timeout, float maxSpeed, const Eigen::Vector3f& gains) {

  GainScheduler tempSched;
  tempSched.addStep(0.0f, gains(0), gains(1), gains(2), 0.0f);

  float initialPos = (target == TuneTarget::THETA_PID) ? chassis->getPose(false).theta : 
                     (target == TuneTarget::X_PID) ? chassis->getPose(false).x : chassis->getPose(false).y;

  if (target == TuneTarget::X_PID) chassis->setXGains({{0.0f, {gains(0), gains(1), gains(2), 0.0f, 0.0f}}});
  if (target == TuneTarget::Y_PID) chassis->setYGains({{0.0f, {gains(0), gains(1), gains(2), 0.0f, 0.0f}}});
  if (target == TuneTarget::THETA_PID) chassis->setThetaGains({{0.0f, {gains(0), gains(1), gains(2), 0.0f, 0.0f}}});

  Pose startPose = chassis->getPose(false);
  float targetVal = initialPos + dist;


  MoveParams params;
  params.timeout = timeout;
  params.maxTranslationSpeed = maxSpeed;
  params.maxRotationSpeed = maxSpeed;
  params.async = true;

  if (target == TuneTarget::THETA_PID) {
    chassis->turnToHeading(std::remainder(targetVal, 360.0f), params);
  } else if (target == TuneTarget::X_PID) {
    float tx = startPose.x + dist * std::sin(startPose.theta * M_PI / 180.0f);
    float ty = startPose.y + dist * std::cos(startPose.theta * M_PI / 180.0f);
    chassis->moveToPoint(tx, ty, params, false);
  } else {
    float tx = startPose.x + dist * std::cos(startPose.theta * M_PI / 180.0f);
    float ty = startPose.y - dist * std::sin(startPose.theta * M_PI / 180.0f);
    chassis->moveToPoint(tx, ty, params, false);
  }


  uint32_t startTime = pros::millis();
  float itae = 0.0f;
  float maxOvershoot = 0.0f;
  uint32_t settleTime = timeout;
  
  while (chassis->motion.isInMotion()) {
    uint32_t elapsed = pros::millis() - startTime;
    float t_sec = elapsed / 1000.0f;
    Pose currentPose = chassis->getPose(false);
    
    float error = 0.0f;
    if (target == TuneTarget::THETA_PID) {
      error = std::abs(getAngleError(targetVal, currentPose.theta));
      if (error > maxOvershoot && elapsed > 500) maxOvershoot = error; 
    } else {
      float currentVal = (target == TuneTarget::X_PID) ? std::hypot(currentPose.x - startPose.x, currentPose.y - startPose.y) :
                                                         std::hypot(currentPose.x - startPose.x, currentPose.y - startPose.y);
      error = std::abs(dist - currentVal);
      if (currentVal > dist) {
         float overshoot = currentVal - dist;
         if (overshoot > maxOvershoot) maxOvershoot = overshoot;
      }
    }
    
    itae += t_sec * error * 0.01f;

    if (error < 1.0f && settleTime == timeout) {
      settleTime = elapsed;
    } else if (error >= 1.0f) {
      settleTime = timeout;
    }
    
    pros::delay(10);
  }

  if (settleTime == timeout) {
    itae += 1000.0f;
  }


  float totalCost = itae + (maxOvershoot * 50.0f) + (settleTime * 0.05f);


  params.async = false;
  if (target == TuneTarget::THETA_PID) {
    chassis->turnToHeading(startPose.theta, params);
  } else {
    chassis->moveToPoint(startPose.x, startPose.y, params, false);
  }
  pros::delay(500);

  return totalCost;
}

void AutoTuner::run(Chassis* chassis, TuneTarget target, TuneConfig config) {
  std::cout << "\n====================================\n";
  std::cout << "Starting Nelder-Mead Auto-Tune...\n";
  std::cout << "Target: " << (target == TuneTarget::X_PID ? "X" : target == TuneTarget::Y_PID ? "Y" : "THETA") << "\n";
  std::cout << "Distance: " << config.dist << "\n";
  std::cout << "Max Cycles: " << config.maxCycles << "\n";
  std::cout << "====================================\n";


  std::vector<SimplexVertex> simplex(4);
  
  if (target == TuneTarget::THETA_PID) {
    simplex[0].point = Eigen::Vector3f(2.0f, 0.0f, 0.1f);
    simplex[1].point = Eigen::Vector3f(3.0f, 0.0f, 0.1f);
    simplex[2].point = Eigen::Vector3f(2.0f, 0.02f, 0.1f);
    simplex[3].point = Eigen::Vector3f(2.0f, 0.0f, 0.3f);
  } else {
    simplex[0].point = Eigen::Vector3f(15.0f, 0.0f, 1.5f);
    simplex[1].point = Eigen::Vector3f(25.0f, 0.0f, 1.5f);
    simplex[2].point = Eigen::Vector3f(15.0f, 0.5f, 1.5f);
    simplex[3].point = Eigen::Vector3f(15.0f, 0.0f, 3.0f);
  }


  for (int i = 0; i < 4; ++i) {
    simplex[i].cost = evaluateGains(chassis, target, config.dist, config.timeout, config.maxSpeed, simplex[i].point);
    std::cout << "Initial " << i << ": [P:" << simplex[i].point(0) << " I:" << simplex[i].point(1) << " D:" << simplex[i].point(2) << "] Cost: " << simplex[i].cost << "\n";
  }


  const float alpha = 1.0f;
  const float gamma = 2.0f;
  const float rho = 0.5f;
  const float sigma = 0.5f;

  for (int cycle = 0; cycle < config.maxCycles; ++cycle) {
    std::sort(simplex.begin(), simplex.end());

    std::cout << "Cycle " << cycle + 1 << " Best: [P:" << simplex[0].point(0) << " I:" << simplex[0].point(1) << " D:" << simplex[0].point(2) << "] Cost: " << simplex[0].cost << "\n";


    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (int i = 0; i < 3; ++i) centroid += simplex[i].point;
    centroid /= 3.0f;


    Eigen::Vector3f reflectedPoint = centroid + alpha * (centroid - simplex[3].point);
    reflectedPoint = reflectedPoint.cwiseMax(Eigen::Vector3f::Zero());
    float reflectedCost = evaluateGains(chassis, target, config.dist, config.timeout, config.maxSpeed, reflectedPoint);

    if (reflectedCost >= simplex[0].cost && reflectedCost < simplex[2].cost) {
      simplex[3].point = reflectedPoint;
      simplex[3].cost = reflectedCost;
      continue;
    }


    if (reflectedCost < simplex[0].cost) {
      Eigen::Vector3f expandedPoint = centroid + gamma * (reflectedPoint - centroid);
      expandedPoint = expandedPoint.cwiseMax(Eigen::Vector3f::Zero());
      float expandedCost = evaluateGains(chassis, target, config.dist, config.timeout, config.maxSpeed, expandedPoint);

      if (expandedCost < reflectedCost) {
        simplex[3].point = expandedPoint;
        simplex[3].cost = expandedCost;
      } else {
        simplex[3].point = reflectedPoint;
        simplex[3].cost = reflectedCost;
      }
      continue;
    }


    Eigen::Vector3f contractedPoint = centroid + rho * (simplex[3].point - centroid);
    contractedPoint = contractedPoint.cwiseMax(Eigen::Vector3f::Zero());
    float contractedCost = evaluateGains(chassis, target, config.dist, config.timeout, config.maxSpeed, contractedPoint);

    if (contractedCost < simplex[3].cost) {
      simplex[3].point = contractedPoint;
      simplex[3].cost = contractedCost;
      continue;
    }


    for (int i = 1; i < 4; ++i) {
      simplex[i].point = simplex[0].point + sigma * (simplex[i].point - simplex[0].point);
      simplex[i].point = simplex[i].point.cwiseMax(Eigen::Vector3f::Zero());
      simplex[i].cost = evaluateGains(chassis, target, config.dist, config.timeout, config.maxSpeed, simplex[i].point);
    }
  }

  std::sort(simplex.begin(), simplex.end());
  
  std::cout << "\n====================================\n";
  std::cout << "Auto-Tune Complete!\n";
  std::cout << "Best Gains for " << (target == TuneTarget::X_PID ? "X" : target == TuneTarget::Y_PID ? "Y" : "THETA") << ":\n";
  std::cout << "kP: " << simplex[0].point(0) << "\n";
  std::cout << "kI: " << simplex[0].point(1) << "\n";
  std::cout << "kD: " << simplex[0].point(2) << "\n";
  std::cout << "Cost: " << simplex[0].cost << "\n";
  std::cout << "====================================\n";
}
