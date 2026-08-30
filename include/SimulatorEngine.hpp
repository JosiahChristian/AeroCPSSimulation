#pragma once

#include "ControlCommandGate.hpp"
#include "FlightModels.hpp"

#include <cstddef>
#include <memory>

class SimulatorEngine {
public:
    SimulatorEngine(
        double targetAltitude,
        double planetaryGravity,
        std::shared_ptr<const ControlCommandGate> commandGate = nullptr);

    void initializeSystem();
    void executeTimeSliceStep(double timeStep);
    [[nodiscard]] bool isTrajectoryTrackingComplete() const noexcept;

    [[nodiscard]] double getCurrentAltitude() const noexcept { return vehicleState_.altitude; }
    [[nodiscard]] double getCurrentVelocity() const noexcept { return vehicleState_.velocity; }
    [[nodiscard]] double getTargetAltitude() const noexcept { return targetAltitude_; }
    [[nodiscard]] const VehicleState& getVehicleState() const noexcept { return vehicleState_; }
    [[nodiscard]] const FlightEnvironment& getEnvironment() const noexcept { return environment_; }
    [[nodiscard]] bool wasLastCommandVetoed() const noexcept { return lastCommandVetoed_; }
    [[nodiscard]] std::size_t getGateVetoCount() const noexcept { return gateVetoCount_; }

private:
    double targetAltitude_;
    VehicleState vehicleState_;
    FlightEnvironment environment_;
    std::shared_ptr<const ControlCommandGate> commandGate_;
    bool systemInitialized_{false};
    bool lastCommandVetoed_{false};
    std::size_t gateVetoCount_{0};
};
