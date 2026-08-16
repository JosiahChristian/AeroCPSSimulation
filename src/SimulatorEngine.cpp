#include "SimulatorEngine.hpp"

#include "NumericalIntegrator.hpp"

#include <cmath>
#include <stdexcept>

namespace {
constexpr double proportionalGain = 0.8;
constexpr double dampingGain = 1.4;
constexpr double altitudeTolerance = 0.25;
constexpr double velocityTolerance = 0.25;
}

SimulatorEngine::SimulatorEngine(double targetAltitude, double planetaryGravity)
    : targetAltitude_(targetAltitude), environment_(planetaryGravity) {
    if (!std::isfinite(targetAltitude_) || targetAltitude_ < 0.0) {
        throw std::invalid_argument("target altitude must be finite and non-negative");
    }
}

void SimulatorEngine::initializeSystem() {
    vehicleState_ = VehicleState{};
    systemInitialized_ = true;
}

void SimulatorEngine::executeTimeSliceStep(double timeStep) {
    if (!systemInitialized_) {
        throw std::logic_error("simulation must be initialized before stepping");
    }
    const double error = targetAltitude_ - vehicleState_.altitude;
    const double gravityCompensation = -environment_.gravity();
    const double thrust = gravityCompensation + proportionalGain * error - dampingGain * vehicleState_.velocity;
    const double acceleration = thrust + environment_.gravity();

    NumericalIntegrator::advanceSemiImplicitEuler(vehicleState_, acceleration, timeStep);

    if (vehicleState_.altitude < 0.0) {
        vehicleState_ = VehicleState{};
    }
}

bool SimulatorEngine::isTrajectoryTrackingComplete() const noexcept {
    return systemInitialized_ &&
           std::abs(targetAltitude_ - vehicleState_.altitude) <= altitudeTolerance &&
           std::abs(vehicleState_.velocity) <= velocityTolerance;
}
