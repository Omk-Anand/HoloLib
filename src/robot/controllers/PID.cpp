#include "PID.h"

// Updated constructors to include kF
PID::PID(double kP, double kI, double kD, double kF, double windupRange, bool signFlipReset)
    : m_gains({kP, kI, kD, kF}),
      m_windupRange(windupRange),
      m_signFlipReset(signFlipReset) {}

PID::PID(const PIDGains& gains, double windupRange, bool signFlipReset)
    : m_gains(gains),
      m_windupRange(windupRange),
      m_signFlipReset(signFlipReset) {}

PIDGains PID::getGains() { 
    return m_gains; 
}

void PID::setGains(PIDGains gains) { 
    m_gains = gains; 
}

double PID::update(double error) {
    uint32_t now = pros::millis();
    double dt = 0;
    
    if (m_initialized) {
        dt = (now - m_previousTime) / 1000.0;
    } else {
        m_initialized = true;
    }
    m_previousTime = now;
    double derivative = (dt > 0) ? (error - m_previousError) / dt : 0;
    m_integral += error * dt;

    if (sgn(error) != sgn(m_previousError) && m_signFlipReset) {
        m_integral = 0;
    }
        if (std::abs(error) > m_windupRange && m_windupRange != 0) {
        m_integral = 0;
    }

    m_previousError = error;

    double feedforward = sgn(error) * m_gains.kF;
    return (error * m_gains.kP) + (m_integral * m_gains.kI) + (derivative * m_gains.kD) + feedforward;
}

void PID::reset() {
    m_previousError = 0;
    m_integral = 0;
    m_initialized = false; 
}

void PID::setSignFlipReset(bool signFlipReset) { 
    m_signFlipReset = signFlipReset; 
}

bool PID::getSignFlipReset() { 
    return m_signFlipReset; 
}

void PID::setWindupRange(double windupRange) { 
    m_windupRange = windupRange; 
}

double PID::getWindupRange() { 
    return m_windupRange; 
}