#include "RigidBodyDynamics.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
bool nearlyEqual(double left, double right, double tolerance = 1e-12) {
    return std::abs(left - right) <= tolerance;
}
}

int main() {
    const RigidBodyProperties properties(2.0, {1.0, 2.0, 4.0});
    RigidBodyState state;

    RigidBodyDynamics::advance(
        state,
        properties,
        {2.0, 4.0, 6.0},
        {1.0, 2.0, 4.0},
        0.5);

    assert(nearlyEqual(state.velocityMetersPerSecond.x, 0.5));
    assert(nearlyEqual(state.velocityMetersPerSecond.y, 1.0));
    assert(nearlyEqual(state.velocityMetersPerSecond.z, 1.5));
    assert(nearlyEqual(state.positionMeters.x, 0.25));
    assert(nearlyEqual(state.positionMeters.y, 0.5));
    assert(nearlyEqual(state.positionMeters.z, 0.75));
    assert(nearlyEqual(state.angularVelocityRadiansPerSecond.x, 0.5));
    assert(nearlyEqual(state.angularVelocityRadiansPerSecond.y, 0.5));
    assert(nearlyEqual(state.angularVelocityRadiansPerSecond.z, 0.5));
    assert(nearlyEqual(state.attitudeBodyToWorld.squaredNorm(), 1.0));

    const double pi = std::acos(-1.0);
    const Quaternion quarterTurn = Quaternion::fromAxisAngle({0.0, 0.0, 1.0}, pi / 2.0);
    const Vector3 rotated = rotateBodyToWorld(quarterTurn, {1.0, 0.0, 0.0});
    assert(nearlyEqual(rotated.x, 0.0));
    assert(nearlyEqual(rotated.y, 1.0));
    assert(nearlyEqual(rotated.z, 0.0));

    RigidBodyState bodyLoadState;
    bodyLoadState.attitudeBodyToWorld = quarterTurn;
    RigidBodyDynamics::advanceBodyLoads(
        bodyLoadState,
        properties,
        {2.0, 0.0, 0.0},
        {},
        0.5);
    assert(nearlyEqual(bodyLoadState.velocityMetersPerSecond.x, 0.0));
    assert(nearlyEqual(bodyLoadState.velocityMetersPerSecond.y, 0.5));
    assert(nearlyEqual(bodyLoadState.velocityMetersPerSecond.z, 0.0));

    RigidBodyState coupledState;
    coupledState.angularVelocityRadiansPerSecond = {1.0, 2.0, 3.0};
    RigidBodyDynamics::advance(coupledState, properties, {}, {}, 0.01);
    assert(nearlyEqual(coupledState.angularVelocityRadiansPerSecond.x, 0.88));
    assert(nearlyEqual(coupledState.angularVelocityRadiansPerSecond.y, 2.045));
    assert(nearlyEqual(coupledState.angularVelocityRadiansPerSecond.z, 2.995));
    assert(nearlyEqual(coupledState.attitudeBodyToWorld.squaredNorm(), 1.0));

    bool rejectedMass = false;
    try {
        static_cast<void>(RigidBodyProperties(0.0, {1.0, 1.0, 1.0}));
    } catch (const std::invalid_argument&) {
        rejectedMass = true;
    }
    assert(rejectedMass);

    bool rejectedInertia = false;
    try {
        static_cast<void>(RigidBodyProperties(1.0, {1.0, -1.0, 1.0}));
    } catch (const std::invalid_argument&) {
        rejectedInertia = true;
    }
    assert(rejectedInertia);

    bool rejectedNonFiniteForce = false;
    try {
        RigidBodyState invalidState;
        RigidBodyDynamics::advance(
            invalidState,
            properties,
            {std::numeric_limits<double>::infinity(), 0.0, 0.0},
            {},
            0.1);
    } catch (const std::invalid_argument&) {
        rejectedNonFiniteForce = true;
    }
    assert(rejectedNonFiniteForce);

    return 0;
}
