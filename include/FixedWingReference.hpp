#pragma once

#include "AerodynamicModel.hpp"
#include "RigidBodyDynamics.hpp"
#include "PitchController.hpp"

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
    [[nodiscard]] static PitchController pitchController();

    static void advanceTrimStep(RigidBodyState& state, double timeStep);
};
