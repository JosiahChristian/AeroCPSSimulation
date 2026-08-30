#include "SimulatorEngine.hpp"

#include <cassert>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace {
bool nearlyEqual(double left, double right, double tolerance = 1e-9) {
    return std::abs(left - right) <= tolerance;
}

class AcceptAllControlGate final : public ControlCommandGate {
public:
    [[nodiscard]] ControlGateDecision evaluate(const ControlCommandProposal&) const override {
        return ControlGateDecision::Accept;
    }

    [[nodiscard]] const char* name() const noexcept override {
        return "accept-all";
    }
};

class VetoAllControlGate final : public ControlCommandGate {
public:
    [[nodiscard]] ControlGateDecision evaluate(const ControlCommandProposal&) const override {
        return ControlGateDecision::Veto;
    }

    [[nodiscard]] const char* name() const noexcept override {
        return "veto-all";
    }
};
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
    assert(!simulation.wasLastCommandVetoed());
    assert(simulation.getGateVetoCount() == 0);

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

    SimulatorEngine acceptedSimulation(
        50.0,
        -9.81,
        std::make_shared<AcceptAllControlGate>());
    acceptedSimulation.initializeSystem();
    acceptedSimulation.executeTimeSliceStep(0.05);
    assert(acceptedSimulation.getCurrentAltitude() > 0.0);
    assert(!acceptedSimulation.wasLastCommandVetoed());

    SimulatorEngine vetoedSimulation(
        50.0,
        -9.81,
        std::make_shared<VetoAllControlGate>());
    vetoedSimulation.initializeSystem();
    const VehicleState beforeVeto = vetoedSimulation.getVehicleState();
    vetoedSimulation.executeTimeSliceStep(0.05);
    assert(vetoedSimulation.wasLastCommandVetoed());
    assert(vetoedSimulation.getGateVetoCount() == 1);
    assert(nearlyEqual(vetoedSimulation.getVehicleState().altitude, beforeVeto.altitude));
    assert(nearlyEqual(vetoedSimulation.getVehicleState().velocity, beforeVeto.velocity));

    return 0;
}
