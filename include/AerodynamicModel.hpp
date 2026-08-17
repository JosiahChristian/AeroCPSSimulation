#pragma once

#include "Quaternion.hpp"
#include "Vector3.hpp"

class AtmosphereModel {
public:
    AtmosphereModel(double seaLevelDensity, double densityScaleHeightMeters, Vector3 windWorldMetersPerSecond = {});

    [[nodiscard]] double densityAtAltitude(double altitudeMeters) const;
    [[nodiscard]] const Vector3& windWorldMetersPerSecond() const noexcept { return windWorld_; }

private:
    double seaLevelDensity_;
    double densityScaleHeightMeters_;
    Vector3 windWorld_;
};

class AerodynamicProperties {
public:
    AerodynamicProperties(double referenceAreaSquareMeters, double dragCoefficient);

    [[nodiscard]] double referenceArea() const noexcept { return referenceAreaSquareMeters_; }
    [[nodiscard]] double dragCoefficient() const noexcept { return dragCoefficient_; }

private:
    double referenceAreaSquareMeters_;
    double dragCoefficient_;
};

class AerodynamicModel {
public:
    struct Loads {
        Vector3 forceBodyNewtons;
        Vector3 momentBodyNewtonMeters;
    };

    struct AirData {
        Vector3 velocityBodyMetersPerSecond;
        double airspeedMetersPerSecond;
        double angleOfAttackRadians;
        double sideslipAngleRadians;
    };

    struct LongitudinalCoefficients {
        double zeroAngleLift;
        double liftSlopePerRadian;
        double zeroLiftDrag;
        double inducedDragFactor;
        double zeroAnglePitchMoment;
        double pitchMomentSlopePerRadian;
        double referenceChordMeters;
    };

    struct LateralDirectionalCoefficients {
        double sideForceBeta;
        double sideForceRollRate;
        double sideForceYawRate;
        double rollMomentBeta;
        double rollMomentRollRate;
        double rollMomentYawRate;
        double yawMomentBeta;
        double yawMomentRollRate;
        double yawMomentYawRate;
        double referenceSpanMeters;
    };

    [[nodiscard]] static Vector3 dragForceWorld(
        Vector3 vehicleVelocityWorldMetersPerSecond,
        double altitudeMeters,
        const AtmosphereModel& atmosphere,
        const AerodynamicProperties& properties);

    [[nodiscard]] static Loads longitudinalLoads(
        double airspeedMetersPerSecond,
        double angleOfAttackRadians,
        double densityKilogramsPerCubicMeter,
        const AerodynamicProperties& properties,
        const LongitudinalCoefficients& coefficients);

    [[nodiscard]] static Loads lateralDirectionalLoads(
        double airspeedMetersPerSecond,
        double sideslipAngleRadians,
        double rollRateRadiansPerSecond,
        double yawRateRadiansPerSecond,
        double densityKilogramsPerCubicMeter,
        const AerodynamicProperties& properties,
        const LateralDirectionalCoefficients& coefficients);

    [[nodiscard]] static Loads combinedLoads(
        double airspeedMetersPerSecond,
        double angleOfAttackRadians,
        double sideslipAngleRadians,
        Vector3 angularVelocityBodyRadiansPerSecond,
        double densityKilogramsPerCubicMeter,
        const AerodynamicProperties& properties,
        const LongitudinalCoefficients& longitudinalCoefficients,
        const LateralDirectionalCoefficients& lateralDirectionalCoefficients);

    [[nodiscard]] static AirData airDataFromWorldState(
        Vector3 vehicleVelocityWorldMetersPerSecond,
        const Quaternion& attitudeBodyToWorld,
        const AtmosphereModel& atmosphere);

    [[nodiscard]] static Loads loadsFromWorldState(
        Vector3 vehicleVelocityWorldMetersPerSecond,
        double altitudeMeters,
        const Quaternion& attitudeBodyToWorld,
        Vector3 angularVelocityBodyRadiansPerSecond,
        const AtmosphereModel& atmosphere,
        const AerodynamicProperties& properties,
        const LongitudinalCoefficients& longitudinalCoefficients,
        const LateralDirectionalCoefficients& lateralDirectionalCoefficients);
};
