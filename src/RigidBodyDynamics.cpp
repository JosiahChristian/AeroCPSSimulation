#include "RigidBodyDynamics.hpp"

#include <cmath>
#include <stdexcept>

RigidBodyProperties::RigidBodyProperties(
    double massKilograms,
    Vector3 diagonalInertiaKilogramMetersSquared)
    : massKilograms_(massKilograms), diagonalInertia_(diagonalInertiaKilogramMetersSquared) {
    if (!std::isfinite(massKilograms_) || massKilograms_ <= 0.0) {
        throw std::invalid_argument("rigid-body mass must be finite and positive");
    }
    if (!diagonalInertia_.isFinite() || diagonalInertia_.x <= 0.0 ||
        diagonalInertia_.y <= 0.0 || diagonalInertia_.z <= 0.0) {
        throw std::invalid_argument("diagonal inertia values must be finite and positive");
    }
}

void RigidBodyDynamics::advance(
    RigidBodyState& state,
    const RigidBodyProperties& properties,
    Vector3 netForceWorldNewtons,
    Vector3 netTorqueBodyNewtonMeters,
    double timeStep) {
    if (!state.positionMeters.isFinite() || !state.velocityMetersPerSecond.isFinite() ||
        !state.attitudeBodyToWorld.isFinite() || !state.angularVelocityRadiansPerSecond.isFinite()) {
        throw std::invalid_argument("rigid-body state must be finite");
    }
    if (!netForceWorldNewtons.isFinite() || !netTorqueBodyNewtonMeters.isFinite()) {
        throw std::invalid_argument("force and torque inputs must be finite");
    }
    if (!std::isfinite(timeStep) || timeStep <= 0.0) {
        throw std::invalid_argument("time step must be finite and positive");
    }

    const Vector3 linearAcceleration = netForceWorldNewtons / properties.mass();
    const Vector3 inertia = properties.diagonalInertia();
    const Vector3 angularMomentum{
        inertia.x * state.angularVelocityRadiansPerSecond.x,
        inertia.y * state.angularVelocityRadiansPerSecond.y,
        inertia.z * state.angularVelocityRadiansPerSecond.z
    };
    const Vector3 effectiveTorque =
        netTorqueBodyNewtonMeters - cross(state.angularVelocityRadiansPerSecond, angularMomentum);
    const Vector3 angularAcceleration{
        effectiveTorque.x / inertia.x,
        effectiveTorque.y / inertia.y,
        effectiveTorque.z / inertia.z
    };

    state.velocityMetersPerSecond += linearAcceleration * timeStep;
    state.positionMeters += state.velocityMetersPerSecond * timeStep;
    state.angularVelocityRadiansPerSecond += angularAcceleration * timeStep;

    const Quaternion angularRate{
        0.0,
        state.angularVelocityRadiansPerSecond.x,
        state.angularVelocityRadiansPerSecond.y,
        state.angularVelocityRadiansPerSecond.z
    };
    const Quaternion derivative = state.attitudeBodyToWorld * angularRate;
    state.attitudeBodyToWorld.w += 0.5 * derivative.w * timeStep;
    state.attitudeBodyToWorld.x += 0.5 * derivative.x * timeStep;
    state.attitudeBodyToWorld.y += 0.5 * derivative.y * timeStep;
    state.attitudeBodyToWorld.z += 0.5 * derivative.z * timeStep;
    state.attitudeBodyToWorld.normalize();
}

void RigidBodyDynamics::advanceBodyLoads(
    RigidBodyState& state,
    const RigidBodyProperties& properties,
    Vector3 netForceBodyNewtons,
    Vector3 netTorqueBodyNewtonMeters,
    double timeStep) {
    if (!state.attitudeBodyToWorld.isFinite() || !netForceBodyNewtons.isFinite()) {
        throw std::invalid_argument("attitude and body force must be finite");
    }

    const Vector3 netForceWorldNewtons =
        rotateBodyToWorld(state.attitudeBodyToWorld, netForceBodyNewtons);
    advance(state, properties, netForceWorldNewtons, netTorqueBodyNewtonMeters, timeStep);
}
