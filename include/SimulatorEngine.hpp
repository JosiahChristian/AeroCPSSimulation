#pragma once

#include "FlightModels.hpp"

class SimulatorEngine {
public:
    SimulatorEngine(double targetAltitude, double planetaryGravity);

    void initializeSystem();
    void executeTimeSliceStep(double timeStep);
    [[nodiscard]] bool isTrajectoryTrackingComplete() const noexcept;

    [[nodiscard]] double getCurrentAltitude() const noexcept { return vehicleState_.altitude; }
    [[nodiscard]] double getCurrentVelocity() const noexcept { return vehicleState_.velocity; }
    [[nodiscard]] double getTargetAltitude() const noexcept { return targetAltitude_; }
    [[nodiscard]] const VehicleState& getVehicleState() const noexcept { return vehicleState_; }
    [[nodiscard]] const FlightEnvironment& getEnvironment() const noexcept { return environment_; }

private:
    double targetAltitude_;
    VehicleState vehicleState_;
    FlightEnvironment environment_;
    bool systemInitialized_{false};
};
