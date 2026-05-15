#pragma once

#include "api.h"
#include "PID.h" 
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <vector>
#include <cmath>
#include <algorithm>

class EncoderKalmanFilter {
public:
    EncoderKalmanFilter(float process_noise = 0.1f, float measurement_noise = 1.0f);
    float update(float measured_position, float dt);

private:
    Eigen::Vector2f x;
    Eigen::Matrix2f P, Q;
    Eigen::RowVector2f H;
    float R;
};


struct ScheduledGain {
    float threshold;
    PIDGains gains;
};

class GainScheduler {
public:
    void addStep(float threshold, float kp, float ki, float kd) {
        schedules.push_back({threshold, {kp, ki, kd}});
        std::sort(schedules.begin(), schedules.end(), [](const ScheduledGain& a, const ScheduledGain& b) {
            return a.threshold < b.threshold;
        });
    }

    PIDGains getGains(float error) {
        float absError = std::abs(error);

        if (schedules.empty()) return {0, 0, 0};
                if (absError <= schedules.front().threshold) {
            return schedules.front().gains;
        }
        if (absError >= schedules.back().threshold) {
            return schedules.back().gains;
        }
        for (size_t i = 0; i < schedules.size() - 1; ++i) {
            const auto& p1 = schedules[i];
            const auto& p2 = schedules[i+1];

            if (absError >= p1.threshold && absError <= p2.threshold) {
                float t = (absError - p1.threshold) / (p2.threshold - p1.threshold);
                float kp = p1.gains.kP + t * (p2.gains.kP - p1.gains.kP);
                float ki = p1.gains.kI + t * (p2.gains.kI - p1.gains.kI);
                float kd = p1.gains.kD + t * (p2.gains.kD - p1.gains.kD);

                return {kp, ki, kd};
            }
        }
        return schedules.back().gains;
    }

private:
    std::vector<ScheduledGain> schedules;
};

struct Pose {
    float x, y, theta;
};

struct XDriveVoltages {
    float fl, fr, bl, br;
};

struct ChassisConfig {
    float trackWidth, wheelDiameter, gearRatio;
};

struct DriveCurve
{
    float curve_multipler = 1;
    float deadzone = 0;
    float minimum_output = 0;
};

struct DriveCurves
{
    DriveCurve movement, rotation;
};

struct MoveParams {
    float maxTranslationSpeed = 12000;
    float maxRotationSpeed = 6000;

    float exitRange = 1.0;

    uint32_t timeout = 3000;

    bool async = false;
};

class Chassis {
public:
    Chassis(pros::Motor fl, pros::Motor fr, pros::Motor bl, pros::Motor br, pros::Imu imu_sensor, ChassisConfig config);
    void setXGains(std::vector<ScheduledGain> steps);
    void setYGains(std::vector<ScheduledGain> steps);
    void setThetaGains(std::vector<ScheduledGain> steps);
    void setPose(float x, float y, float theta);
    Pose getPose(bool radians = false);
    void brake();
    void setMotorVoltages(XDriveVoltages v);
    XDriveVoltages calculateHolonomic(float vx, float vy, float vt);

    void moveToPoint(float x, float y, MoveParams params = {}, bool AngleCorrection = false);
    void turnToHeading(float targetDeg, MoveParams params = {});
    void turnToPoint(float tx, float ty, MoveParams params = {});
    void moveToPose(float tx, float ty, float targetThetaDeg, MoveParams params = {});
    void swing(float targetThetaDeg, bool leftSide, MoveParams params = {});
    void curveCircle(float targetThetaDeg, float radius, MoveParams params = {});
    void driveControl(float forward_power, float sideways_power, float rotational_power, DriveCurves drivecurves);

private:
    pros::Motor frontLeft, frontRight, backLeft, backRight;
    pros::Imu imu;
    ChassisConfig config;
    GainScheduler xSched, ySched, thetaSched;
    Pose currentPose{0, 0, 0};
    pros::Mutex poseMutex;
    EncoderKalmanFilter kf_fl, kf_fr, kf_bl, kf_br;
    float prev_fl = 0, prev_fr = 0, prev_bl = 0, prev_br = 0, prev_heading = 0;
    void odometryTask();
    friend void odomTaskTrampoline(void* param);
};