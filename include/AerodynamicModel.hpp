#pragma once

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
    struct LongitudinalLoads {
        Vector3 forceBodyNewtons;
        Vector3 momentBodyNewtonMeters;
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

    [[nodiscard]] static Vector3 dragForceWorld(
        Vector3 vehicleVelocityWorldMetersPerSecond,
        double altitudeMeters,
        const AtmosphereModel& atmosphere,
        const AerodynamicProperties& properties);

    [[nodiscard]] static LongitudinalLoads longitudinalLoads(
        double airspeedMetersPerSecond,
        double angleOfAttackRadians,
        double densityKilogramsPerCubicMeter,
        const AerodynamicProperties& properties,
        const LongitudinalCoefficients& coefficients);
};
