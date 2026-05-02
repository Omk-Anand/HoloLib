#include "chassis.h"
#include <algorithm> // For std::clamp, std::max

Chassis::Chassis(std::vector<std::int8_t> rightMotorsPortList, std::vector<std::int8_t> leftMotorsPortList)
    : rightMotorPorts(rightMotorsPortList), leftMotorPorts(leftMotorsPortList),
      rightMotors(rightMotorsPortList), leftMotors(leftMotorsPortList),
      controller(config.kV, config.KA_straight, config.KA_turn, config.KS_straight, config.KS_turn, config.KP_straight, config.KI_straight, integralWindup, trackWidth) 
{
    ltv = new LTVPathFollower(this);
}

Chassis::~Chassis() {
    delete ltv;
}

void Chassis::tank(float lin_vel, float ang_vel, const VelocityControllerConfig &config, unsigned int time) {
    u_int32_t starting_time = pros::millis();
    
    while(pros::millis() - starting_time < time) {
        float leftVel = leftMotors.get_actual_velocity() * rpmToMpsConversion;
        float rightVel = rightMotors.get_actual_velocity() * rpmToMpsConversion;
        
        DrivetrainVoltages outputVoltages = controller.update(lin_vel, ang_vel, leftVel, rightVel);
    
        leftMotors.move_voltage(outputVoltages.leftVoltage * 1000);
        rightMotors.move_voltage(outputVoltages.rightVoltage * 1000);
    
        pros::delay(10);
    }
    brake(); 
}

void Chassis::setDrivetrainConstants(float wheel_diameter_inches, float gear_ratio, float track_width_inches) {
    wheelDiameter = wheel_diameter_inches;
    gearRatio = gear_ratio;
    trackWidth = track_width_inches * INCH_TO_METER;
    rpmToMpsConversion = (gearRatio / 60.0f) * (M_PI * wheelDiameter * INCH_TO_METER);
    controller = VoltageController(config.kV, config.KA_straight, config.KA_turn, config.KS_straight, 
                                   config.KS_turn, config.KP_straight, config.KI_straight, 
                                   integralWindup, trackWidth);
}

void Chassis::setVelocityController(float kV, float KA_turn, float KA_straight, float KS_turn, 
                                    float KS_straight, float KP_straight, float KI_straight, 
                                    float integral_windup) {
    config.kV = kV;
    config.KA_turn = KA_turn;
    config.KA_straight = KA_straight;
    config.KS_turn = KS_turn;
    config.KS_straight = KS_straight;
    config.KP_straight = KP_straight;
    config.KI_straight = KI_straight;
    integralWindup = integral_windup;
    controller = VoltageController(config.kV, config.KA_straight, config.KA_turn, config.KS_straight, 
                                   config.KS_turn, config.KP_straight, config.KI_straight, 
                                   integralWindup, trackWidth);
}

void Chassis::setLTVParameters(float q_x, float q_y, float q_theta, float r_vel, float r_ang, 
                               float max_lin_corr, float max_ang_corr) {
    defaultLtvConfig.q_x = q_x;
    defaultLtvConfig.q_y = q_y;
    defaultLtvConfig.q_theta = q_theta;
    defaultLtvConfig.r_vel = r_vel;
    defaultLtvConfig.r_ang = r_ang;
    defaultLtvConfig.max_lin_correction = max_lin_corr;
    defaultLtvConfig.max_ang_correction = max_ang_corr;
}

void Chassis::setLTVBackwardsParameters(float q_x_b, float q_y_b, float q_theta_b, float r_vel_b, float r_ang_b) {
    defaultLtvConfig.q_x_b = q_x_b;
    defaultLtvConfig.q_y_b = q_y_b;
    defaultLtvConfig.q_theta_b = q_theta_b;
    defaultLtvConfig.r_vel_b = r_vel_b;
    defaultLtvConfig.r_ang_b = r_ang_b;
}

void Chassis::executeVelocityCommand(float v_cmd, float w_cmd) {
    float left_actual_mps = leftMotors.get_actual_velocity() * rpmToMpsConversion;
    float right_actual_mps = rightMotors.get_actual_velocity() * rpmToMpsConversion;

    DrivetrainVoltages output_voltages = controller.update(v_cmd, w_cmd, left_actual_mps, right_actual_mps);

    auto clamp = [](double val, double min, double max) {
        if (val < min) return min;
        if (val > max) return max;
        return val;
    };

    output_voltages.rightVoltage = clamp(output_voltages.rightVoltage, -12.0, 12.0);
    output_voltages.leftVoltage = clamp(output_voltages.leftVoltage, -12.0, 12.0);

    rightMotors.move_voltage(output_voltages.rightVoltage * 1000.0);
    leftMotors.move_voltage(output_voltages.leftVoltage * 1000.0);
}

void Chassis::followPath(const Path& path) {
    ltv->followPath(path, defaultLtvConfig);
}

void Chassis::followPath(const Path& path, const LTVPathFollower::ltvConfig& l_config) {
    ltv->followPath(path, l_config);
}

void Chassis::waitUntilDone() { ltv->waitUntilDone(); }
void Chassis::waitUntil(float dist_inches) { ltv->waitUntil(dist_inches); }
void Chassis::waitUntil(float x_inch, float y_inch, float radius_inch) { ltv->waitUntil(x_inch, y_inch, radius_inch); }

// --- RAMSETE INTEGRATION ---

void Chassis::moveToPointRamsete(float target_x, float target_y, int timeout_msec, const VelocityControllerConfig &config, MoveToPointParams params, bool async) {
    
    this->requestMotionStart(); 
    this->distTraveled = 0; 

    auto movement_logic = [=, this]() mutable {
        target_x *= INCH_TO_METER;
        target_y *= INCH_TO_METER;
        float earlyExit_m = params.earlyExitRange * INCH_TO_METER;
        
        VoltageController controller(
            config.kV, config.KA_straight, config.KA_turn,
            config.KS_straight, config.KS_turn,
            config.KP_straight, config.KI_straight,
            5000.0, this->trackWidth // Modular track width
        );

        lemlib::PID angularPID(11, 0.001, 0);
        lemlib::PID lateralPID(5, 0, 0);
        lemlib::ExitCondition lateral(0.08, 100);
        lemlib::ExitCondition longitudinal(0.08, 100);
        
        float lateralGain = 0;
        
        auto sign = [](float x) { return (x > 0) ? 1.0f : ((x < 0) ? -1.0f : 0.0f); };
        auto sinc = [](float x) { return (std::abs(x) < 1e-5) ? 1.0f : std::sin(x) / x; };

        bool is_settling = false; 

        Pose lastPose = this->getPose(); // Using custom Pose struct
        uint32_t startTime = pros::millis();

        while ((pros::millis() - startTime < static_cast<uint32_t>(timeout_msec)) && (!lateral.getExit() || !longitudinal.getExit())) {
            
            if (this->motionQueued) {
                break;
            } 
            
            Pose currentPoseRaw = this->getPose();
            this->distTraveled += lastPose.distance(currentPoseRaw);
            lastPose = currentPoseRaw;

            Pose currentPose = currentPoseRaw;
            currentPose.x *= INCH_TO_METER;
            currentPose.y *= INCH_TO_METER;
            
            float d = std::hypot(target_x - currentPose.x, target_y - currentPose.y);
            float dx = target_x - currentPose.x;
            float dy = target_y - currentPose.y;

            if (!params.forwards) {
                dx = -dx;
                dy = -dy;
            }

            float theta = currentPose.theta;
            float localErrorX = std::cos(theta) * dx + std::sin(theta) * dy;
            float localErrorY = -std::sin(theta) * dx + std::cos(theta) * dy;

            float angularError = std::atan2(localErrorY, localErrorX);
            float cosineScaling = std::cos(angularError);
            float driveError;

            if (d < 0.05f) {
                is_settling = true;
            }

            if (is_settling) { 
                angularError = 0;     
                cosineScaling = 1.0f; 
                driveError = std::cos(theta) * dx + std::sin(theta) * dy;
            } else {
                driveError = d * sign(cosineScaling); 
            }

            if (params.minSpeed > 0 && (d < earlyExit_m || driveError < 0)) {
                break;
            }

            lateral.update(localErrorY);
            longitudinal.update(localErrorX);

            float angularOutput = angularPID.update(angularError);
            float driveOutput = lateralPID.update(driveError);
            
            driveOutput = std::clamp(driveOutput, -params.maxSpeed, params.maxSpeed);
            driveOutput = driveOutput * std::abs(cosineScaling);

            if (is_settling) { 
                angularOutput = 0; 
            } else {
                angularOutput += driveOutput * lateralGain * localErrorY * sinc(angularError);
            }

            if (!is_settling && params.minSpeed > 0 && std::abs(driveOutput) < params.minSpeed) {
                driveOutput = params.minSpeed * sign(driveOutput);
                if (driveOutput == 0) driveOutput = params.minSpeed;
            }

            angularOutput = std::clamp(angularOutput, -params.maxTurnSpeed, params.maxTurnSpeed);
        
            if (!params.forwards) {
                driveOutput = -driveOutput;
            }

            float leftVel = leftMotors.get_actual_velocity() * rpmToMpsConversion;
            float rightVel = rightMotors.get_actual_velocity() * rpmToMpsConversion;
            
            DrivetrainVoltages outputVoltages = controller.update(driveOutput, angularOutput, leftVel, rightVel);
            
            leftMotors.move_voltage(outputVoltages.leftVoltage * 1000);
            rightMotors.move_voltage(outputVoltages.rightVoltage * 1000);

            pros::delay(10);
        }

        if (params.minSpeed == 0) {
            leftMotors.move_voltage(0);
            rightMotors.move_voltage(0);
        }
        
        this->motionRunning = false;
    };

    if (async) {
        pros::Task task(movement_logic);
    } else {
        movement_logic();
    }
}

void Chassis::moveToPoseRamsete(float targetX, float targetY, float targetTheta, int timeout_msec, const VelocityControllerConfig &config, MoveToPoseParams params, bool async) {
    
    if (async) {
        pros::Task task([=, this]() {
            moveToPoseRamsete(targetX, targetY, targetTheta, timeout_msec, config, params, false);
        });
        pros::delay(10); 
        return;          
    }

    this->requestMotionStart(); 
    this->distTraveled = 0; 

    float settleRadius_m = params.settleRadius * INCH_TO_METER;
    float earlyExit_m = params.earlyExitRange * INCH_TO_METER; 

    VoltageController controller(
        config.kV, config.KA_straight, config.KA_turn,
        config.KS_straight, config.KS_turn,
        config.KP_straight, config.KI_straight,
        5000.0, this->trackWidth // Modular track width
    );

    lemlib::PID angularPID(7.5, 0.001, 26.5);
    lemlib::PID lateralPID(5.5 , 0.001 , 27);
    
    lemlib::ExitCondition lateralExit(0.08, 100);
    lemlib::ExitCondition longitudinalExit(0.08, 100);

    auto sign_func = [](float x) { return (x > 0) ? 1.0f : ((x < 0) ? -1.0f : 0.0f); };
    auto sinc = [](float x) { return (std::abs(x) < 1e-5) ? 1.0f : std::sin(x) / x; };

    float targetThetaRad = targetTheta * (M_PI / 180.0f);
    float end_unit_x = std::sin(targetThetaRad);
    float end_unit_y = std::cos(targetThetaRad);
    float dir_sign = params.forwards ? 1.0f : -1.0f;

    bool is_settling = false;

    Pose lastPose = this->getPose(); // Custom Pose struct
    uint32_t startTime = pros::millis();

    while ((pros::millis() - startTime < static_cast<uint32_t>(timeout_msec)) && (!lateralExit.getExit() || !longitudinalExit.getExit())) {
        
        if (this->motionQueued) {
            break;
        }
        
        Pose currentPoseRaw = this->getPose();
        this->distTraveled += lastPose.distance(currentPoseRaw);
        lastPose = currentPoseRaw;

        Pose currentPose = currentPoseRaw;
        currentPose.x *= INCH_TO_METER;
        currentPose.y *= INCH_TO_METER;
        float theta = currentPose.theta;

        float d = std::hypot(targetX - currentPose.x, targetY - currentPose.y);

        float true_dx = targetX - currentPose.x;
        float true_dy = targetY - currentPose.y;
        if (!params.forwards) {
            true_dx = -true_dx;
            true_dy = -true_dy;
        }
        float true_localErrorX = std::cos(theta) * true_dx + std::sin(theta) * true_dy;
        float true_localErrorY = -std::sin(theta) * true_dx + std::cos(theta) * true_dy;

        float rx = currentPose.x - targetX;
        float ry = currentPose.y - targetY;
        float along_track = rx * end_unit_x + ry * end_unit_y;
        float dynamic_lookahead = d * params.lead;
        float lookahead_m = std::max(dynamic_lookahead, 0.35f);
        float ghost_along_track = along_track + (lookahead_m * dir_sign);
        
        float carrot_x = targetX + ghost_along_track * end_unit_x;
        float carrot_y = targetY + ghost_along_track * end_unit_y;

        float dx = carrot_x - currentPose.x;
        float dy = carrot_y - currentPose.y;
        if (!params.forwards) {
            dx = -dx;
            dy = -dy;
        }

        float localErrorX = std::cos(theta) * dx + std::sin(theta) * dy;
        float localErrorY = -std::sin(theta) * dx + std::cos(theta) * dy;

        float heading_error_rad = std::atan2(localErrorY, localErrorX);
        float cosine_scale = std::cos(heading_error_rad);

        float error_norm_sq = localErrorX * localErrorX + localErrorY * localErrorY;
        float driveError = std::sqrt((error_norm_sq + 2.0f * d * d) / 3.0f) * sign_func(cosine_scale);
        float turnError = heading_error_rad;

        if (d < std::max(settleRadius_m, 0.05f)) {
            is_settling = true;
        }

        if (is_settling) {
            float targetThetaMath = M_PI_2 - targetThetaRad;
            float final_error = targetThetaMath - theta;
            final_error = std::remainder(final_error, 2.0 * M_PI);
            
            turnError = final_error;
            driveError = true_localErrorX; 
            cosine_scale = 1.0f; 
        }

        if (params.earlyExitRange > 0 && (d < earlyExit_m || driveError < 0)) {
            break; 
        }

        lateralExit.update(true_localErrorY);
        longitudinalExit.update(true_localErrorX); 

        float driveOutput = lateralPID.update(driveError);
        float turnOutput = angularPID.update(turnError);

        driveOutput = std::clamp(driveOutput, -params.maxSpeed, params.maxSpeed);

        if (!is_settling) {
            float steeringVelocity = std::max(std::abs(driveOutput), 0.2f);
            turnOutput += steeringVelocity * params.k_lat * localErrorY * sinc(heading_error_rad);
        }

        driveOutput = std::abs(cosine_scale) * driveOutput;
        
        if (!is_settling && params.minSpeed > 0 && std::abs(driveOutput) < params.minSpeed) {
            driveOutput = params.minSpeed * sign_func(driveOutput);
            if (driveOutput == 0) driveOutput = params.minSpeed;
        }

        if (!params.forwards) {
            driveOutput = -driveOutput; 
        }

        turnOutput = std::clamp(turnOutput, -params.maxTurnSpeed, params.maxTurnSpeed);

        float leftVel = leftMotors.get_actual_velocity() * rpmToMpsConversion;
        float rightVel = rightMotors.get_actual_velocity() * rpmToMpsConversion;
        
        DrivetrainVoltages outputVoltages = controller.update(driveOutput, turnOutput, leftVel, rightVel);
        
        leftMotors.move_voltage(outputVoltages.leftVoltage * 1000);
        rightMotors.move_voltage(outputVoltages.rightVoltage * 1000);

        pros::delay(10);
    }

    if (params.minSpeed == 0) {
        leftMotors.move_voltage(0);
        rightMotors.move_voltage(0);
    }
    
    this->motionRunning = false;
}

