#pragma once

#include "AerodynamicModel.hpp"
#include "RigidBodyDynamics.hpp"

class FixedWingReference {
public:
    static constexpr double trimAirspeedMetersPerSecond = 20.0;
    static constexpr double trimAltitudeMeters = 100.0;
    static constexpr double trimAngleOfAttackRadians = 0.05;
    static constexpr double gravityMetersPerSecondSquared = 9.81;

    [[nodiscard]] static RigidBodyProperties rigidBodyProperties();
    [[nodiscard]] static AerodynamicProperties aerodynamicProperties();
    [[nodiscard]] static AerodynamicModel::LongitudinalCoefficients longitudinalCoefficients();
    [[nodiscard]] static AerodynamicModel::LateralDirectionalCoefficients lateralDirectionalCoefficients();
    [[nodiscard]] static AtmosphereModel atmosphere();
    [[nodiscard]] static RigidBodyState trimState();
    [[nodiscard]] static double trimThrustNewtons();

    static void advanceTrimStep(RigidBodyState& state, double timeStep);
};
