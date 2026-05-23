#pragma once

#include "api.h"
#include "PID.h"
#include "motion_handler.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <functional>
#include <limits>

class PoseEKF {
private:
    Eigen::Vector3f x; // State: [x, y, theta]
    Eigen::Matrix3f P; // Covariance Matrix
    Eigen::Matrix3f Q; // Process Noise Matrix
    Eigen::Matrix<float, 1, 3> H; // Observation Matrix (for absolute heading)
    float R;           // Measurement Noise (IMU absolute heading)

public:
    PoseEKF(float initial_x, float initial_y, float initial_theta) {
        x << initial_x, initial_y, initial_theta;
        
        P.setZero(); // Start with high certainty of initial pose
        
        // Process noise: How much we trust our encoder/IMU deltas
        // Tune these based on your robot's slip!
        Q << 0.01f, 0,     0,
             0,     0.01f, 0,
             0,     0,     0.001f; 
             
        // We only directly measure absolute theta from the IMU in the update step
        H << 0, 0, 1;
        R = 0.05f; // Trust in the absolute IMU reading
    }

    // Prediction Step: Using SE(2) Pose Exponentiation
    void predict(float dx_local, float dy_local, float dtheta) {
        float s, c;
        if (std::abs(dtheta) < 1e-4f) {
            // Taylor series expansion for numerical stability near zero
            s = 1.0f - (dtheta * dtheta) / 6.0f;
            c = dtheta / 2.0f;
        } else {
            // Exact arc integration
            s = std::sin(dtheta) / dtheta;
            c = (1.0f - std::cos(dtheta)) / dtheta;
        }

        // Apply constant curvature to local deltas
        float dx_arc = dx_local * s - dy_local * c;
        float dy_arc = dx_local * c + dy_local * s;

        float prev_theta = x(2);
        float cos_t = std::cos(prev_theta);
        float sin_t = std::sin(prev_theta);

        // Rotate into global frame
        float dx_global =  dx_arc * cos_t - dy_arc * sin_t;
        float dy_global =  dx_arc * sin_t + dy_arc * cos_t;

        // Update State (f(x, u))
        x(0) += dx_global;
        x(1) += dy_global;
        x(2) += dtheta;

        // Wrap heading to [-PI, PI]
        x(2) = std::remainder(x(2), 2.0f * M_PI);

        // Compute Jacobian (F)
        Eigen::Matrix3f F = Eigen::Matrix3f::Identity();
        F(0, 2) = -dy_global;
        F(1, 2) =  dx_global;

        // Propagate Covariance
        P = F * P * F.transpose() + Q;
    }

    // Update Step: Fusing absolute IMU measurement
    void updateIMU(float measured_theta) {
        // Innovation (Measurement residual)
        float y = measured_theta - x(2);
        y = std::remainder(y, 2.0f * M_PI); // Normalize error

        // Innovation covariance
        float S = (H * P * H.transpose())(0, 0) + R;
        
        // Kalman Gain
        Eigen::Vector3f K = P * H.transpose() / S;

        // Update State
        x = x + K * y;
        x(2) = std::remainder(x(2), 2.0f * M_PI); // Wrap again just in case

        // Update Covariance
        P = (Eigen::Matrix3f::Identity() - K * H) * P;
    }

    // Getters
    float getX() const { return x(0); }
    float getY() const { return x(1); }
    float getTheta() const { return x(2); }
    
    void setPose(float new_x, float new_y, float new_theta) {
        x << new_x, new_y, new_theta;
        P.setZero(); // Reset uncertainty on manual set
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Data Structures
// ─────────────────────────────────────────────────────────────────────────────

struct Pose {
    float x     = 0;
    float y     = 0;
    float theta = 0; // stored in RADIANS internally; getPose(false) converts to degrees
};

struct PathPoint {
    float x     = 0;
    float y     = 0;
    float theta = 0; // degrees
};

struct XDriveVoltages {
    float fl, fr, bl, br;
};

struct ChassisConfig {
    float trackWidth;
    float wheelDiameter;
    float gearRatio;
    bool  kfEnabled = true;
};

struct DriveCurve {
    float curve_multipler  = 1.0f;
    float deadzone         = 0.0f;
    float minimum_output   = 0.0f;
};

struct DriveCurves {
    DriveCurve movement;
    DriveCurve rotation;
};

struct MoveParams {
    float    maxTranslationSpeed = 127.0f;  // [-127, 127] — calculateHolonomic scales to mV
    float    maxRotationSpeed    = 127.0f;
    float    minSpeed            = 0.0f;
    float    exitRange           = 1.0f;    // inches / degrees
    float    earlyExitRange      = 0.0f;    // instantly skip when within this range
    uint32_t timeout             = 3000;    // ms
    bool     async               = true;
};


struct ScheduledGain {
    float    threshold;
    PIDGains gains;
};

// ─────────────────────────────────────────────────────────────────────────────
// Path Parser — free function, usable anywhere
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Parse a CSV string (or file path) of "x, y, theta" rows into a PathPoint vector.
 * @param input_source  Raw multi-line CSV string literal  OR  a file path string.
 * @param convertFromMeters  If true, x/y are multiplied by 39.37 (m → inches).
 */
std::vector<PathPoint> parsePathData(const std::string& input_source,
                                     bool               convertFromMeters = false);

// ─────────────────────────────────────────────────────────────────────────────
// Gain Scheduler
// ─────────────────────────────────────────────────────────────────────────────

class GainScheduler {
public:
    /** Add a breakpoint.  Steps are auto-sorted by threshold ascending. */
    void addStep(float threshold, float kP, float kI, float kD);

    /**
     * Return interpolated gains for the given error magnitude.
     * - Below the lowest threshold  → lowest-threshold gains
     * - Above the highest threshold → highest-threshold gains
     * - Between two thresholds      → linearly interpolated
     */
    PIDGains getGains(float error) const;

    void clear();

private:
    std::vector<ScheduledGain> schedules;
};

// ─────────────────────────────────────────────────────────────────────────────
// Encoder Kalman Filter
// ─────────────────────────────────────────────────────────────────────────────

class EncoderKalmanFilter {
public:
    EncoderKalmanFilter(float process_noise     = 0.077f,
                        float measurement_noise = 7.2f);
    float update(float measured_position, float dt);

private:
    Eigen::Vector2f    x;
    Eigen::Matrix2f    P, Q;
    Eigen::RowVector2f H;
    float              R;
};

// ─────────────────────────────────────────────────────────────────────────────
// Chassis
// ─────────────────────────────────────────────────────────────────────────────

class Chassis {
public:
    // ── Construction ──────────────────────────────────────────────────────────
    Chassis(pros::Motor fl, pros::Motor fr,
            pros::Motor bl, pros::Motor br,
            pros::Imu imu_sensor, ChassisConfig config);

    // ── Gain Configuration ────────────────────────────────────────────────────
    void setXGains    (std::vector<ScheduledGain> steps);
    void setYGains    (std::vector<ScheduledGain> steps);
    void setThetaGains(std::vector<ScheduledGain> steps);

    // ── Pose ──────────────────────────────────────────────────────────────────
    /**
     * Override the current odometry pose.
     * @param theta  Degrees.
     */
    void setPose(float x, float y, float theta = 0.0f);

    /**
     * @param radians  If false (default), theta is returned in degrees.
     */
    Pose getPose(bool radians = false);

    // ── Low-level Drive ───────────────────────────────────────────────────────
    /**
     * Holonomic mixing.  Inputs are in [-127, 127]; output is millivolts.
     */
    XDriveVoltages calculateHolonomic(float vx, float vy, float vt);
    void           setMotorVoltages  (XDriveVoltages v);
    void           brake             ();

    // ── Operator Control ──────────────────────────────────────────────────────
    void driveControl(float forward, float sideways, float rotation,
                      DriveCurves drivecurves);

    // ── Autonomous Movements ──────────────────────────────────────────────────
    

    enum class HeadingMode {
        FollowPath,
        HoldAngle
    };

    void followPathPID(
    const std::vector<PathPoint>& path,
    float                         lookahead_inches,
    MoveParams                    params = {},
    HeadingMode                   headingMode = HeadingMode::FollowPath,
    float                         holdAngleDeg = 0.0f,
    bool                          reversed = false
);

    /** Turn in-place to an absolute heading (degrees). */
    void turnToHeading(float targetDeg, MoveParams params = {});

    /** Turn in-place to face a field-relative point. */
    void turnToPoint(float tx, float ty, MoveParams params = {});

    /** Translate to a point while holding current heading. */
    void moveToPoint(float tx, float ty,
                     MoveParams params          = {},
                     bool       angleCorrection = false);

    /** Translate + rotate to a full pose simultaneously. */
    void moveToPose(float tx, float ty, float targetThetaDeg,
                    MoveParams params = {});

    /** Arc (constant-radius curve) to a target heading. */
    void curveCircle(float targetThetaDeg, float radius,
                     MoveParams params = {});

    // ── Motion Handler (public so callers can do chassis.motion.cancelAll()) ──
    MotionHandler motion;

    // ── Odometry Task (called internally; do not call directly) ──────────────
    void odometryTask();

private:
    pros::Motor frontLeft, frontRight, backLeft, backRight;
    pros::Imu   imu;
    ChassisConfig config;

    GainScheduler xSched, ySched, thetaSched;

    Pose        currentPose{0, 0, 0};
    pros::Mutex poseMutex;
    PoseEKF     ekf{0, 0, 0}; // Start at origin with 0 heading
    EncoderKalmanFilter kf_fl, kf_fr, kf_bl, kf_br;
    float prev_fl = 0, prev_fr = 0, prev_bl = 0, prev_br = 0;
    float prev_heading = 0;

    friend void odomTaskTrampoline(void*);
};

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

/** Shortest signed angle error in degrees, result in (-180, 180]. */
inline float getAngleError(float target, float current) {
    return std::remainder(target - current, 360.0f);
}