#include "SimulatorEngine.hpp"

#include <cmath>
#include <stdexcept>

namespace {
constexpr double proportionalGain = 0.8;
constexpr double dampingGain = 1.4;
constexpr double altitudeTolerance = 0.25;
constexpr double velocityTolerance = 0.25;
}

SimulatorEngine::SimulatorEngine(double targetAltitude, double planetaryGravity)
    : targetAltitude_(targetAltitude), gravityConstant_(planetaryGravity) {
    if (!std::isfinite(targetAltitude_) || targetAltitude_ < 0.0) {
        throw std::invalid_argument("target altitude must be finite and non-negative");
    }
    if (!std::isfinite(gravityConstant_) || gravityConstant_ >= 0.0) {
        throw std::invalid_argument("planetary gravity must be finite and negative");
    }
}

void SimulatorEngine::initializeSystem() {
    currentAltitude_ = 0.0;
    currentVelocity_ = 0.0;
    systemInitialized_ = true;
}

void SimulatorEngine::executeTimeSliceStep(double timeStep) {
    if (!systemInitialized_) {
        throw std::logic_error("simulation must be initialized before stepping");
    }
    if (!std::isfinite(timeStep) || timeStep <= 0.0) {
        throw std::invalid_argument("time step must be finite and positive");
    }

    const double error = targetAltitude_ - currentAltitude_;
    const double gravityCompensation = -gravityConstant_;
    const double thrust = gravityCompensation + proportionalGain * error - dampingGain * currentVelocity_;
    const double acceleration = thrust + gravityConstant_;

    currentVelocity_ += acceleration * timeStep;
    currentAltitude_ += currentVelocity_ * timeStep;

    if (currentAltitude_ < 0.0) {
        currentAltitude_ = 0.0;
        currentVelocity_ = 0.0;
    }
}

bool SimulatorEngine::isTrajectoryTrackingComplete() const noexcept {
    return systemInitialized_ &&
           std::abs(targetAltitude_ - currentAltitude_) <= altitudeTolerance &&
           std::abs(currentVelocity_) <= velocityTolerance;
}
