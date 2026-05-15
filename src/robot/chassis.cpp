#include "chassis.h"

void odomTaskTrampoline(void* param) {
    static_cast<Chassis*>(param)->odometryTask();
}

EncoderKalmanFilter::EncoderKalmanFilter(float process_noise, float measurement_noise) {
    x.setZero(); 
    P.setIdentity();
    Q << process_noise, 0, 0, process_noise;
    R = measurement_noise;
    H << 1, 0; 
}

float EncoderKalmanFilter::update(float measured_position, float dt) {
    Eigen::Matrix2f F;
    F << 1, dt, 0, 1;
    x = F * x;
    P = F * P * F.transpose() + Q;

    float y = measured_position - H * x; 
    float S = H * P * H.transpose() + R; 
    Eigen::Vector2f K = P * H.transpose() / S; 

    x = x + K * y;
    P = (Eigen::Matrix2f::Identity() - K * H) * P;

    return x(0); 
}
Chassis::Chassis(pros::Motor fl, pros::Motor fr, pros::Motor bl, pros::Motor br, 
                 pros::Imu imu_sensor, ChassisConfig config)
    : frontLeft(fl), 
      frontRight(fr), 
      backLeft(bl), 
      backRight(br), 
      imu(imu_sensor), 
      config(config) 
{
    prev_fl = frontLeft.get_position();
    prev_fr = frontRight.get_position();
    prev_bl = backLeft.get_position();
    prev_br = backRight.get_position();
    prev_heading = imu.get_heading() * (M_PI / 180.0f);

    pros::Task odom_task(odomTaskTrampoline, this, "Odometry Task");
}

void Chassis::odometryTask() {
    uint32_t now = pros::millis();
    const float dt = 0.01f;
    
    const float d_per_deg = (M_PI * config.wheelDiameter * config.gearRatio) / 360.0f;

    Eigen::Matrix<float, 2, 4> kinematics;
    kinematics << 1, -1, -1, 1, 1, 1, 1, 1;
    kinematics /= 4.0f;

    while (true) {
        float c_fl = kf_fl.update(frontLeft.get_position(), dt);
        float c_fr = kf_fr.update(frontRight.get_position(), dt);
        float c_bl = kf_bl.update(backLeft.get_position(), dt);
        float c_br = kf_br.update(backRight.get_position(), dt);
        float c_heading = imu.get_heading() * (M_PI / 180.0f);

        Eigen::Vector4f deltas((c_fl - prev_fl) * d_per_deg, 
                              (c_fr - prev_fr) * d_per_deg, 
                              (c_bl - prev_bl) * d_per_deg, 
                              (c_br - prev_br) * d_per_deg);
        
        float d_theta = c_heading - prev_heading;
        Eigen::Vector2f local_delta = kinematics * deltas;
        
        float s, c;
        if (std::abs(d_theta) < 1e-4) {
            s = 1.0f - (d_theta * d_theta) / 6.0f;
            c = d_theta / 2.0f;
        } else {
            s = std::sin(d_theta) / d_theta;
            c = (1.0f - std::cos(d_theta)) / d_theta;
        }

        Eigen::Matrix2f mat; mat << s, -c, c, s;
        Eigen::Vector2f chord = mat * local_delta;

        Eigen::Vector2f global_d = Eigen::Rotation2D<float>(prev_heading) * chord;

        poseMutex.take();
        currentPose.x += global_d.x();
        currentPose.y += global_d.y();
        currentPose.theta = c_heading;
        poseMutex.give();

        prev_fl = c_fl; prev_fr = c_fr; prev_bl = c_bl; prev_br = c_br; prev_heading = c_heading;
        pros::Task::delay_until(&now, 10);
    }
}

XDriveVoltages Chassis::calculateHolonomic(float vx, float vy, float vt) {
    XDriveVoltages v;
    v.fl = vy + vx + vt;
    v.fr = vy - vx - vt;
    v.bl = vy - vx + vt;
    v.br = vy + vx - vt;

    float max = std::max({std::abs(v.fl), std::abs(v.fr), std::abs(v.bl), std::abs(v.br), 12000.0f});
    if (max > 12000.0f) {
        float ratio = 12000.0f / max;
        v.fl *= ratio; v.fr *= ratio; v.bl *= ratio; v.br *= ratio;
    }
    return v;
}

void Chassis::setMotorVoltages(XDriveVoltages v) {
    frontLeft.move_voltage(v.fl);
    frontRight.move_voltage(v.fr);
    backLeft.move_voltage(v.bl);
    backRight.move_voltage(v.br);
}

void Chassis::brake() {
    frontLeft.brake();
    frontRight.brake();
    backLeft.brake();
    backRight.brake();
}

void Chassis::setPose(float x, float y, float theta) {
    poseMutex.take();
    currentPose = {x, y, theta};
    prev_heading = theta * (M_PI / 180.0f);
    poseMutex.give();
}

Pose Chassis::getPose(bool radians) {
    poseMutex.take();
    Pose p = currentPose;
    poseMutex.give();
    if (!radians) p.theta *= (180.0f / M_PI);
    return p;
}

void Chassis::setXGains(std::vector<ScheduledGain> steps) {
    xSched = GainScheduler(); 
    for (const auto& step : steps) xSched.addStep(step.threshold, step.gains.kP, step.gains.kI, step.gains.kD);
}

void Chassis::setYGains(std::vector<ScheduledGain> steps) {
    ySched = GainScheduler(); 
    for (const auto& step : steps) ySched.addStep(step.threshold, step.gains.kP, step.gains.kI, step.gains.kD);
}

void Chassis::setThetaGains(std::vector<ScheduledGain> steps) {
    thetaSched = GainScheduler(); 
    for (const auto& step : steps) thetaSched.addStep(step.threshold, step.gains.kP, step.gains.kI, step.gains.kD);
}


float getAngleError(float target, float current) {
    return std::remainder(target - current, 360.0f);
}

void Chassis::turnToHeading(float targetDeg, MoveParams params) {
    if (params.async) {
        pros::Task([=, this]() { turnToHeading(targetDeg, params); });
        return;
    }

    uint32_t start = pros::millis();
    uint32_t settleStart = 0;
    const uint32_t settleTime = 100;
    float prevError = 0;
    PID tPID(0, 0, 0, 0); 

    while (pros::millis() - start < params.timeout) {
        float currentDeg = getPose(false).theta;
        float error = getAngleError(targetDeg, currentDeg);

        if (std::abs(error) < params.exitRange) {
            if (settleStart == 0) settleStart = pros::millis();
            
        float velocity = (error - prevError) / 0.01f;
        if (pros::millis() - settleStart > settleTime && std::abs(velocity) < 0.5f) break;
        } else {
            settleStart = 0;
        }

        PIDGains g = thetaSched.getGains(error);
        tPID.setGains(g);
        float output = tPID.update(error);
        
        setMotorVoltages(calculateHolonomic(0, 0, output));
        pros::delay(10);
        prevError = error;
    }
    brake();
}

void Chassis::turnToPoint(float tx, float ty, MoveParams params) {
    Pose p = getPose(false);
    float targetAngle = std::atan2(ty - p.y, tx - p.x) * (180.0f / M_PI);
    turnToHeading(targetAngle, params);
}

void Chassis::moveToPoint(float tx, float ty, MoveParams params, bool angleCorrection) {

    if (params.async) {

        pros::Task([=, this]() { 

            moveToPoint(tx, ty, {params.maxTranslationSpeed, params.maxRotationSpeed, params.exitRange, params.timeout, false}, angleCorrection); 

        });

        return;

    }



    uint32_t start = pros::millis();

    uint32_t settleStart = 0;

    const uint32_t settleTime = 100; 



    PID xPID(0, 0, 0, 0); 

    PID yPID(0, 0, 0, 0); 

    PID tPID(0, 0, 0, 0); 



    while (pros::millis() - start < params.timeout) {

        Pose curr = getPose(false);

        float ex = tx - curr.x;

        float ey = ty - curr.y;

        float distError = std::hypot(ex, ey);




        if (distError < params.exitRange) {

            if (settleStart == 0) settleStart = pros::millis();

            if (pros::millis() - settleStart > settleTime) break;

        } else {

            settleStart = 0;

        }

        float targetAngle = std::atan2(ey, ex) * (180.0f / M_PI);
        float angleError = getAngleError(targetAngle, curr.theta);

        PIDGains gx = xSched.getGains(ex);

        PIDGains gy = ySched.getGains(ey);

        PIDGains gt = thetaSched.getGains(angleError);

        xPID.setGains(gx);

        yPID.setGains(gy);

        tPID.setGains(gt);
        

        float outX_global = xPID.update(ex); 
        float outY_global = yPID.update(ey);
        float outTheta = tPID.update(angleError);
        float rad = -curr.theta * (M_PI / 180.0f);
        float cos_t = std::cos(rad);
        float sin_t = std::sin(rad);

        float outX_local = outX_global * cos_t - outY_global * sin_t;
        float outY_local = outX_global * sin_t + outY_global * cos_t;
        if (distError < 2.0f) outTheta = 0;



        setMotorVoltages(calculateHolonomic(outX_local, outY_local, angleCorrection ? outTheta : 0));

        

        pros::delay(10);

    }

    brake();

}



void Chassis::moveToPose(
    float tx,
    float ty,
    float targetThetaDeg,
    MoveParams params
) {
    if (params.async) {
        pros::Task([=, this]() {
            moveToPose(
                tx,
                ty,
                targetThetaDeg,
                {
                    params.maxTranslationSpeed,
                    params.maxRotationSpeed,
                    params.exitRange,
                    params.timeout,
                    false
                }
            );
        });

        return;
    }
    constexpr float DEG2RAD = M_PI / 180.0f;

    constexpr uint32_t settleTime = 120;

    constexpr float angleExitRange = 2.0f;

    constexpr float kS_translation = 5.0f;
    constexpr float kS_rotation = 4.0f;

    uint32_t startTime = pros::millis();
    uint32_t settleStart = 0;

    PID xPID(0,0,0,0);
    PID yPID(0,0,0,0);
    PID thetaPID(0,0,0,0);

    while (pros::millis() - startTime < params.timeout) {

        Pose curr = getPose(false);
        float ex = tx - curr.x;
        float ey = ty - curr.y;

        float distError = std::hypot(ex, ey);

        float angleError =
            getAngleError(targetThetaDeg, curr.theta);

        bool positionSettled =
            distError < params.exitRange;

        bool angleSettled =
            std::abs(angleError) < angleExitRange;

        if (positionSettled && angleSettled) {

            if (settleStart == 0)
                settleStart = pros::millis();

            if (pros::millis() - settleStart >= settleTime)
                break;

        } else {
            settleStart = 0;
        }

        PIDGains gx = xSched.getGains(ex);
        PIDGains gy = ySched.getGains(ey);
        PIDGains gt = thetaSched.getGains(angleError);

        xPID.setGains(gx);
        yPID.setGains(gy);
        thetaPID.setGains(gt);

        float outX_global = xPID.update(ex);
        float outY_global = yPID.update(ey);
        if (std::abs(outX_global) > 0.01f) {
            outX_global +=
                std::copysign(kS_translation, outX_global);
        }

        if (std::abs(outY_global) > 0.01f) {
            outY_global +=
                std::copysign(kS_translation, outY_global);
        }

        float translationMagnitude =
            std::hypot(outX_global, outY_global);

        if (translationMagnitude >
            params.maxTranslationSpeed) {

            float scale =
                params.maxTranslationSpeed /
                translationMagnitude;

            outX_global *= scale;
            outY_global *= scale;
        }

        float outTheta =
            thetaPID.update(angleError);
        if (std::abs(outTheta) > 0.01f) {
            outTheta +=
                std::copysign(kS_rotation, outTheta);
        }
        float turnScale = std::clamp(
            distError / 24.0f,
            0.2f,
            1.0f
        );

        outTheta *= turnScale;
        outTheta = std::clamp(
            outTheta,
            -params.maxRotationSpeed,
            params.maxRotationSpeed
        );

        float thetaRad =
            -curr.theta * DEG2RAD;

        float cosTheta = std::cos(thetaRad);
        float sinTheta = std::sin(thetaRad);

        float outX_local =
            outX_global * cosTheta -
            outY_global * sinTheta;

        float outY_local =
            outX_global * sinTheta +
            outY_global * cosTheta;
        setMotorVoltages(
            calculateHolonomic(
                outX_local,
                outY_local,
                outTheta
            )
        );

        pros::delay(10);
    }

    brake();
}

void Chassis::swing(float targetThetaDeg, bool leftSide, MoveParams params) {}
void Chassis::curveCircle(float targetThetaDeg, float radius, MoveParams params) {}


void Chassis::driveControl(
    float forward,
    float sideways,
    float rotation,
    DriveCurves drivecurves
) {

    auto applyCurve = [](float x, const DriveCurve& c) {
        if (std::abs(x) < c.deadzone)
            return 0.0f;
        if (x > 0) x = (x - c.deadzone) / (1.0f - c.deadzone);
        else        x = (x + c.deadzone) / (1.0f - c.deadzone);
        float sign = (x >= 0) ? 1.0f : -1.0f;
        x = std::pow(std::abs(x), c.curve_multipler) * sign;
        if (std::abs(x) > 0.0f && std::abs(x) < c.minimum_output) {
            x = sign * c.minimum_output;
        }

        return x;
    };

    forward  = applyCurve(forward,  drivecurves.movement);
    sideways = applyCurve(sideways, drivecurves.movement);
    rotation = applyCurve(rotation, drivecurves.rotation);
    XDriveVoltages v;

    v.fl = forward + sideways + rotation;
    v.fr = forward - sideways - rotation;
    v.bl = forward - sideways + rotation;
    v.br = forward + sideways - rotation;
    float maxMag = std::max({
        std::abs(v.fl),
        std::abs(v.fr),
        std::abs(v.bl),
        std::abs(v.br),
        12000.0f
    });

    if (maxMag > 12000.0f) {
        float scale = 12000.0f / maxMag;

        v.fl *= scale;
        v.fr *= scale;
        v.bl *= scale;
        v.br *= scale;
    }
    setMotorVoltages(v);
}