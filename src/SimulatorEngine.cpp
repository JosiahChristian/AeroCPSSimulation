#include "SimulatorEngine.hpp"

#include "NumericalIntegrator.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace {
constexpr double proportionalGain = 0.8;
constexpr double dampingGain = 1.4;
constexpr double altitudeTolerance = 0.25;
constexpr double velocityTolerance = 0.25;
}

SimulatorEngine::SimulatorEngine(
    double targetAltitude,
    double planetaryGravity,
    std::shared_ptr<const ControlCommandGate> commandGate)
    : targetAltitude_(targetAltitude),
      environment_(planetaryGravity),
      commandGate_(std::move(commandGate)) {
    if (!std::isfinite(targetAltitude_) || targetAltitude_ < 0.0) {
        throw std::invalid_argument("target altitude must be finite and non-negative");
    }
}

void SimulatorEngine::initializeSystem() {
    vehicleState_ = VehicleState{};
    systemInitialized_ = true;
    lastCommandVetoed_ = false;
    gateVetoCount_ = 0;
}

void SimulatorEngine::executeTimeSliceStep(double timeStep) {
    if (!systemInitialized_) {
        throw std::logic_error("simulation must be initialized before stepping");
    }
    if (!std::isfinite(timeStep) || timeStep <= 0.0) {
        throw std::invalid_argument("time step must be finite and positive");
    }

    const double error = targetAltitude_ - vehicleState_.altitude;
    const double gravityCompensation = -environment_.gravity();
    const double thrust = gravityCompensation + proportionalGain * error - dampingGain * vehicleState_.velocity;

    lastCommandVetoed_ = false;
    if (commandGate_) {
        const ControlCommandProposal proposal{vehicleState_, targetAltitude_, thrust};
        if (commandGate_->evaluate(proposal) == ControlGateDecision::Veto) {
            lastCommandVetoed_ = true;
            ++gateVetoCount_;
            return;
        }
    }

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
