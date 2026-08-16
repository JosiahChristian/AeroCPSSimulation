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
