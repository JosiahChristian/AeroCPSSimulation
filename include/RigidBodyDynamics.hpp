#pragma once

#include "Quaternion.hpp"
#include "Vector3.hpp"

struct RigidBodyState {
    Vector3 positionMeters;
    Vector3 velocityMetersPerSecond;
    Quaternion attitudeBodyToWorld;
    Vector3 angularVelocityRadiansPerSecond;
};

class RigidBodyProperties {
public:
    RigidBodyProperties(double massKilograms, Vector3 diagonalInertiaKilogramMetersSquared);

    [[nodiscard]] double mass() const noexcept { return massKilograms_; }
    [[nodiscard]] const Vector3& diagonalInertia() const noexcept { return diagonalInertia_; }

private:
    double massKilograms_;
    Vector3 diagonalInertia_;
};

class RigidBodyDynamics {
public:
    static void advance(
        RigidBodyState& state,
        const RigidBodyProperties& properties,
        Vector3 netForceWorldNewtons,
        Vector3 netTorqueBodyNewtonMeters,
        double timeStep);
};
