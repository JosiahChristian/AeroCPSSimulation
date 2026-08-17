#include "AerodynamicModel.hpp"

#include <cmath>
#include <stdexcept>

AtmosphereModel::AtmosphereModel(
    double seaLevelDensity,
    double densityScaleHeightMeters,
    Vector3 windWorldMetersPerSecond)
    : seaLevelDensity_(seaLevelDensity),
      densityScaleHeightMeters_(densityScaleHeightMeters),
      windWorld_(windWorldMetersPerSecond) {
    if (!std::isfinite(seaLevelDensity_) || seaLevelDensity_ <= 0.0) {
        throw std::invalid_argument("sea-level density must be finite and positive");
    }
    if (!std::isfinite(densityScaleHeightMeters_) || densityScaleHeightMeters_ <= 0.0) {
        throw std::invalid_argument("density scale height must be finite and positive");
    }
    if (!windWorld_.isFinite()) {
        throw std::invalid_argument("wind velocity must be finite");
    }
}

double AtmosphereModel::densityAtAltitude(double altitudeMeters) const {
    if (!std::isfinite(altitudeMeters) || altitudeMeters < 0.0) {
        throw std::invalid_argument("altitude must be finite and non-negative");
    }
    return seaLevelDensity_ * std::exp(-altitudeMeters / densityScaleHeightMeters_);
}

AerodynamicProperties::AerodynamicProperties(double referenceAreaSquareMeters, double dragCoefficient)
    : referenceAreaSquareMeters_(referenceAreaSquareMeters), dragCoefficient_(dragCoefficient) {
    if (!std::isfinite(referenceAreaSquareMeters_) || referenceAreaSquareMeters_ <= 0.0) {
        throw std::invalid_argument("reference area must be finite and positive");
    }
    if (!std::isfinite(dragCoefficient_) || dragCoefficient_ < 0.0) {
        throw std::invalid_argument("drag coefficient must be finite and non-negative");
    }
}

Vector3 AerodynamicModel::dragForceWorld(
    Vector3 vehicleVelocityWorldMetersPerSecond,
    double altitudeMeters,
    const AtmosphereModel& atmosphere,
    const AerodynamicProperties& properties) {
    if (!vehicleVelocityWorldMetersPerSecond.isFinite()) {
        throw std::invalid_argument("vehicle velocity must be finite");
    }

    const Vector3 relativeVelocity =
        vehicleVelocityWorldMetersPerSecond - atmosphere.windWorldMetersPerSecond();
    const double speed = std::sqrt(
        relativeVelocity.x * relativeVelocity.x +
        relativeVelocity.y * relativeVelocity.y +
        relativeVelocity.z * relativeVelocity.z);
    if (speed == 0.0 || properties.dragCoefficient() == 0.0) {
        return {};
    }

    const double dynamicPressure = 0.5 * atmosphere.densityAtAltitude(altitudeMeters) * speed * speed;
    const double dragMagnitude =
        dynamicPressure * properties.dragCoefficient() * properties.referenceArea();
    return relativeVelocity * (-dragMagnitude / speed);
}

AerodynamicModel::Loads AerodynamicModel::longitudinalLoads(
    double airspeedMetersPerSecond,
    double angleOfAttackRadians,
    double densityKilogramsPerCubicMeter,
    const AerodynamicProperties& properties,
    const LongitudinalCoefficients& coefficients) {
    if (!std::isfinite(airspeedMetersPerSecond) || airspeedMetersPerSecond < 0.0 ||
        !std::isfinite(angleOfAttackRadians) ||
        !std::isfinite(densityKilogramsPerCubicMeter) || densityKilogramsPerCubicMeter <= 0.0 ||
        !std::isfinite(coefficients.zeroAngleLift) ||
        !std::isfinite(coefficients.liftSlopePerRadian) ||
        !std::isfinite(coefficients.zeroLiftDrag) || coefficients.zeroLiftDrag < 0.0 ||
        !std::isfinite(coefficients.inducedDragFactor) || coefficients.inducedDragFactor < 0.0 ||
        !std::isfinite(coefficients.zeroAnglePitchMoment) ||
        !std::isfinite(coefficients.pitchMomentSlopePerRadian) ||
        !std::isfinite(coefficients.referenceChordMeters) || coefficients.referenceChordMeters <= 0.0) {
        throw std::invalid_argument("longitudinal aerodynamic inputs must be finite and physically valid");
    }

    const double dynamicPressure = 0.5 * densityKilogramsPerCubicMeter *
                                   airspeedMetersPerSecond * airspeedMetersPerSecond;
    const double liftCoefficient = coefficients.zeroAngleLift +
                                   coefficients.liftSlopePerRadian * angleOfAttackRadians;
    const double dragCoefficient = coefficients.zeroLiftDrag +
                                   coefficients.inducedDragFactor * liftCoefficient * liftCoefficient;
    const double pitchMomentCoefficient = coefficients.zeroAnglePitchMoment +
                                          coefficients.pitchMomentSlopePerRadian * angleOfAttackRadians;
    const double area = properties.referenceArea();

    return {
        {-dynamicPressure * area * dragCoefficient, 0.0, dynamicPressure * area * liftCoefficient},
        {0.0, dynamicPressure * area * coefficients.referenceChordMeters * pitchMomentCoefficient, 0.0}
    };
}

AerodynamicModel::Loads AerodynamicModel::lateralDirectionalLoads(
    double airspeedMetersPerSecond,
    double sideslipAngleRadians,
    double rollRateRadiansPerSecond,
    double yawRateRadiansPerSecond,
    double densityKilogramsPerCubicMeter,
    const AerodynamicProperties& properties,
    const LateralDirectionalCoefficients& coefficients) {
    if (!std::isfinite(airspeedMetersPerSecond) || airspeedMetersPerSecond < 0.0 ||
        !std::isfinite(sideslipAngleRadians) ||
        !std::isfinite(rollRateRadiansPerSecond) ||
        !std::isfinite(yawRateRadiansPerSecond) ||
        !std::isfinite(densityKilogramsPerCubicMeter) || densityKilogramsPerCubicMeter <= 0.0 ||
        !std::isfinite(coefficients.sideForceBeta) ||
        !std::isfinite(coefficients.sideForceRollRate) ||
        !std::isfinite(coefficients.sideForceYawRate) ||
        !std::isfinite(coefficients.rollMomentBeta) ||
        !std::isfinite(coefficients.rollMomentRollRate) ||
        !std::isfinite(coefficients.rollMomentYawRate) ||
        !std::isfinite(coefficients.yawMomentBeta) ||
        !std::isfinite(coefficients.yawMomentRollRate) ||
        !std::isfinite(coefficients.yawMomentYawRate) ||
        !std::isfinite(coefficients.referenceSpanMeters) || coefficients.referenceSpanMeters <= 0.0) {
        throw std::invalid_argument(
            "lateral-directional aerodynamic inputs must be finite and physically valid");
    }

    if (airspeedMetersPerSecond == 0.0) {
        return {};
    }

    const double dynamicPressure = 0.5 * densityKilogramsPerCubicMeter *
                                   airspeedMetersPerSecond * airspeedMetersPerSecond;
    const double rateScale = coefficients.referenceSpanMeters /
                             (2.0 * airspeedMetersPerSecond);
    const double normalizedRollRate = rollRateRadiansPerSecond * rateScale;
    const double normalizedYawRate = yawRateRadiansPerSecond * rateScale;
    const double sideForceCoefficient = coefficients.sideForceBeta * sideslipAngleRadians +
        coefficients.sideForceRollRate * normalizedRollRate +
        coefficients.sideForceYawRate * normalizedYawRate;
    const double rollMomentCoefficient = coefficients.rollMomentBeta * sideslipAngleRadians +
        coefficients.rollMomentRollRate * normalizedRollRate +
        coefficients.rollMomentYawRate * normalizedYawRate;
    const double yawMomentCoefficient = coefficients.yawMomentBeta * sideslipAngleRadians +
        coefficients.yawMomentRollRate * normalizedRollRate +
        coefficients.yawMomentYawRate * normalizedYawRate;
    const double forceScale = dynamicPressure * properties.referenceArea();
    const double momentScale = forceScale * coefficients.referenceSpanMeters;

    return {
        {0.0, forceScale * sideForceCoefficient, 0.0},
        {momentScale * rollMomentCoefficient, 0.0, momentScale * yawMomentCoefficient}
    };
}

AerodynamicModel::Loads AerodynamicModel::combinedLoads(
    double airspeedMetersPerSecond,
    double angleOfAttackRadians,
    double sideslipAngleRadians,
    Vector3 angularVelocityBodyRadiansPerSecond,
    double densityKilogramsPerCubicMeter,
    const AerodynamicProperties& properties,
    const LongitudinalCoefficients& longitudinalCoefficients,
    const LateralDirectionalCoefficients& lateralDirectionalCoefficients) {
    if (!angularVelocityBodyRadiansPerSecond.isFinite()) {
        throw std::invalid_argument("body angular velocity must be finite");
    }

    const Loads longitudinal = longitudinalLoads(
        airspeedMetersPerSecond,
        angleOfAttackRadians,
        densityKilogramsPerCubicMeter,
        properties,
        longitudinalCoefficients);
    const Loads lateralDirectional = lateralDirectionalLoads(
        airspeedMetersPerSecond,
        sideslipAngleRadians,
        angularVelocityBodyRadiansPerSecond.x,
        angularVelocityBodyRadiansPerSecond.z,
        densityKilogramsPerCubicMeter,
        properties,
        lateralDirectionalCoefficients);

    Loads combined = longitudinal;
    combined.forceBodyNewtons += lateralDirectional.forceBodyNewtons;
    combined.momentBodyNewtonMeters += lateralDirectional.momentBodyNewtonMeters;
    return combined;
}
