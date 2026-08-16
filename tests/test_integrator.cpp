#include "FlightModels.hpp"
#include "NumericalIntegrator.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>

namespace {
double integratePosition(double timeStep) {
    constexpr double acceleration = 2.0;
    constexpr double duration = 1.0;
    VehicleState state;
    const int steps = static_cast<int>(duration / timeStep);
    for (int step = 0; step < steps; ++step) {
        NumericalIntegrator::advanceSemiImplicitEuler(state, acceleration, timeStep);
    }
    assert(std::abs(state.velocity - 2.0) < 1e-12);
    return state.altitude;
}
}

int main() {
    constexpr double analyticalPosition = 1.0;
    const double coarseError = std::abs(integratePosition(0.1) - analyticalPosition);
    const double mediumError = std::abs(integratePosition(0.05) - analyticalPosition);
    const double fineError = std::abs(integratePosition(0.025) - analyticalPosition);

    assert(mediumError < coarseError);
    assert(fineError < mediumError);
    assert(std::abs(coarseError / mediumError - 2.0) < 1e-9);
    assert(std::abs(mediumError / fineError - 2.0) < 1e-9);

    FlightEnvironment mars(-3.711);
    assert(std::abs(mars.gravity() + 3.711) < 1e-12);

    bool rejectedGravity = false;
    try {
        static_cast<void>(FlightEnvironment(9.81));
    } catch (const std::invalid_argument&) {
        rejectedGravity = true;
    }
    assert(rejectedGravity);

    bool rejectedStep = false;
    try {
        VehicleState state;
        NumericalIntegrator::advanceSemiImplicitEuler(state, 1.0, 0.0);
    } catch (const std::invalid_argument&) {
        rejectedStep = true;
    }
    assert(rejectedStep);

    return 0;
}
