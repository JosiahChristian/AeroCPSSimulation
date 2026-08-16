#include "SimulatorEngine.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>

namespace {
bool nearlyEqual(double left, double right, double tolerance = 1e-9) {
    return std::abs(left - right) <= tolerance;
}
}

int main() {
    SimulatorEngine simulation(50.0, -9.81);
    assert(nearlyEqual(simulation.getEnvironment().gravity(), -9.81));
    assert(nearlyEqual(simulation.getCurrentAltitude(), 0.0));
    assert(nearlyEqual(simulation.getCurrentVelocity(), 0.0));
    assert(!simulation.isTrajectoryTrackingComplete());

    bool rejectedUninitializedStep = false;
    try {
        simulation.executeTimeSliceStep(0.05);
    } catch (const std::logic_error&) {
        rejectedUninitializedStep = true;
    }
    assert(rejectedUninitializedStep);

    simulation.initializeSystem();
    assert(nearlyEqual(simulation.getVehicleState().altitude, 0.0));
    simulation.executeTimeSliceStep(0.05);
    assert(simulation.getCurrentAltitude() > 0.0);
    assert(simulation.getCurrentVelocity() > 0.0);

    for (int step = 0; step < 499 && !simulation.isTrajectoryTrackingComplete(); ++step) {
        simulation.executeTimeSliceStep(0.05);
    }
    assert(simulation.isTrajectoryTrackingComplete());
    assert(std::abs(simulation.getTargetAltitude() - simulation.getCurrentAltitude()) <= 0.25);

    bool rejectedInvalidStep = false;
    try {
        simulation.executeTimeSliceStep(0.0);
    } catch (const std::invalid_argument&) {
        rejectedInvalidStep = true;
    }
    assert(rejectedInvalidStep);

    return 0;
}
