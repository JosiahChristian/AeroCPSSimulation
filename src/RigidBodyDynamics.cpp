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
        !state.orientationRadians.isFinite() || !state.angularVelocityRadiansPerSecond.isFinite()) {
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
    const Vector3 angularAcceleration{
        netTorqueBodyNewtonMeters.x / inertia.x,
        netTorqueBodyNewtonMeters.y / inertia.y,
        netTorqueBodyNewtonMeters.z / inertia.z
    };

    state.velocityMetersPerSecond += linearAcceleration * timeStep;
    state.positionMeters += state.velocityMetersPerSecond * timeStep;
    state.angularVelocityRadiansPerSecond += angularAcceleration * timeStep;
    state.orientationRadians += state.angularVelocityRadiansPerSecond * timeStep;
}
