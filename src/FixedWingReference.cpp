#include "FixedWingReference.hpp"

#include <cmath>

namespace {
constexpr double massKilograms = 8.0;
constexpr double referenceAreaSquareMeters = 0.8;
constexpr double seaLevelDensity = 1.225;
constexpr double densityScaleHeightMeters = 8500.0;
}

RigidBodyProperties FixedWingReference::rigidBodyProperties() {
    return {massKilograms, {1.2, 1.8, 2.4}};
}

AerodynamicProperties FixedWingReference::aerodynamicProperties() {
    return {referenceAreaSquareMeters, 0.0};
}

AtmosphereModel FixedWingReference::atmosphere() {
    return {seaLevelDensity, densityScaleHeightMeters};
}

AerodynamicModel::LongitudinalCoefficients FixedWingReference::longitudinalCoefficients() {
    const double density = atmosphere().densityAtAltitude(trimAltitudeMeters);
    const double dynamicPressure = 0.5 * density * trimAirspeedMetersPerSecond *
                                   trimAirspeedMetersPerSecond;
    const double requiredLiftCoefficient =
        massKilograms * gravityMetersPerSecondSquared * std::cos(trimAngleOfAttackRadians) /
        (dynamicPressure * referenceAreaSquareMeters);
    constexpr double zeroAngleLift = 0.2;
    return {
        zeroAngleLift,
        (requiredLiftCoefficient - zeroAngleLift) / trimAngleOfAttackRadians,
        0.03,
        0.05,
        0.025,
        -0.5,
        0.4,
        0.35,
        1.2
    };
}

AerodynamicModel::LateralDirectionalCoefficients
FixedWingReference::lateralDirectionalCoefficients() {
    return {
        -0.7, 0.0, 0.2,
        -0.12, -0.4, 0.08,
        0.18, -0.03, -0.15,
        2.4
    };
}

RigidBodyState FixedWingReference::trimState() {
    return {
        {0.0, 0.0, trimAltitudeMeters},
        {trimAirspeedMetersPerSecond, 0.0, 0.0},
        Quaternion::fromAxisAngle({0.0, 1.0, 0.0}, trimAngleOfAttackRadians),
        {}
    };
}

double FixedWingReference::trimThrustNewtons() {
    const auto loads = AerodynamicModel::loadsFromWorldState(
        trimState().velocityMetersPerSecond,
        trimAltitudeMeters,
        trimState().attitudeBodyToWorld,
        {},
        atmosphere(),
        aerodynamicProperties(),
        longitudinalCoefficients(),
        lateralDirectionalCoefficients());
    return -loads.forceBodyNewtons.x -
           std::tan(trimAngleOfAttackRadians) * loads.forceBodyNewtons.z;
}

PitchController FixedWingReference::pitchController() {
    return {8.0, 4.0, 2.0};
}

void FixedWingReference::advanceTrimStep(RigidBodyState& state, double timeStep) {
    auto loads = AerodynamicModel::loadsFromWorldState(
        state.velocityMetersPerSecond,
        state.positionMeters.z,
        state.attitudeBodyToWorld,
        state.angularVelocityRadiansPerSecond,
        atmosphere(),
        aerodynamicProperties(),
        longitudinalCoefficients(),
        lateralDirectionalCoefficients());
    loads.forceBodyNewtons.x += trimThrustNewtons();

    Vector3 forceWorld = rotateBodyToWorld(state.attitudeBodyToWorld, loads.forceBodyNewtons);
    forceWorld += {0.0, 0.0, -massKilograms * gravityMetersPerSecondSquared};
    RigidBodyDynamics::advance(
        state, rigidBodyProperties(), forceWorld, loads.momentBodyNewtonMeters, timeStep);
}
