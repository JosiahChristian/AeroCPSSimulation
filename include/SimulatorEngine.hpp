#pragma once

class SimulatorEngine {
public:
    SimulatorEngine(double targetAltitude, double planetaryGravity);

    void initializeSystem();
    void executeTimeSliceStep(double timeStep);
    [[nodiscard]] bool isTrajectoryTrackingComplete() const noexcept;

    [[nodiscard]] double getCurrentAltitude() const noexcept { return currentAltitude_; }
    [[nodiscard]] double getCurrentVelocity() const noexcept { return currentVelocity_; }
    [[nodiscard]] double getTargetAltitude() const noexcept { return targetAltitude_; }

private:
    double targetAltitude_;
    double currentAltitude_{0.0};
    double currentVelocity_{0.0};
    double gravityConstant_;
    bool systemInitialized_{false};
};
