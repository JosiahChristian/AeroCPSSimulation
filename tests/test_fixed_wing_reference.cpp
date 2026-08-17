#include "FixedWingReference.hpp"

#include <cassert>
#include <cmath>

namespace {
bool nearlyEqual(double left, double right, double tolerance = 1e-9) {
    return std::abs(left - right) <= tolerance;
}
}

int main() {
    RigidBodyState state = FixedWingReference::trimState();
    const auto airData = AerodynamicModel::airDataFromWorldState(
        state.velocityMetersPerSecond,
        state.attitudeBodyToWorld,
        FixedWingReference::atmosphere());
    assert(nearlyEqual(airData.airspeedMetersPerSecond,
                       FixedWingReference::trimAirspeedMetersPerSecond));
    assert(nearlyEqual(airData.angleOfAttackRadians,
                       FixedWingReference::trimAngleOfAttackRadians));
    assert(nearlyEqual(airData.sideslipAngleRadians, 0.0));
    assert(FixedWingReference::trimThrustNewtons() > 0.0);

    constexpr double timeStep = 0.01;
    constexpr int steps = 1000;
    for (int step = 0; step < steps; ++step) {
        FixedWingReference::advanceTrimStep(state, timeStep);
    }

    assert(nearlyEqual(state.positionMeters.x,
                       FixedWingReference::trimAirspeedMetersPerSecond * timeStep * steps, 1e-4));
    assert(nearlyEqual(state.positionMeters.y, 0.0));
    assert(nearlyEqual(state.positionMeters.z, FixedWingReference::trimAltitudeMeters, 1e-4));
    assert(nearlyEqual(state.velocityMetersPerSecond.x,
                       FixedWingReference::trimAirspeedMetersPerSecond, 1e-5));
    assert(nearlyEqual(state.velocityMetersPerSecond.y, 0.0));
    assert(nearlyEqual(state.velocityMetersPerSecond.z, 0.0, 1e-5));
    assert(nearlyEqual(state.angularVelocityRadiansPerSecond.x, 0.0));
    assert(nearlyEqual(state.angularVelocityRadiansPerSecond.y, 0.0, 1e-5));
    assert(nearlyEqual(state.angularVelocityRadiansPerSecond.z, 0.0));
    assert(nearlyEqual(state.attitudeBodyToWorld.squaredNorm(), 1.0));

    return 0;
}
