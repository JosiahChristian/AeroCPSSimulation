#include "NumericalIntegrator.hpp"

#include <cmath>
#include <stdexcept>

void NumericalIntegrator::advanceSemiImplicitEuler(
    VehicleState& state,
    double acceleration,
    double timeStep) {
    if (!std::isfinite(acceleration)) {
        throw std::invalid_argument("acceleration must be finite");
    }
    if (!std::isfinite(timeStep) || timeStep <= 0.0) {
        throw std::invalid_argument("time step must be finite and positive");
    }

    state.velocity += acceleration * timeStep;
    state.altitude += state.velocity * timeStep;
}
