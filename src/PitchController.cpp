#include "PitchController.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

PitchController::PitchController(
    double proportionalGainNewtonMetersPerRadian,
    double derivativeGainNewtonMeterSecondsPerRadian,
    double maximumMomentNewtonMeters)
    : proportionalGain_(proportionalGainNewtonMetersPerRadian),
      derivativeGain_(derivativeGainNewtonMeterSecondsPerRadian),
      maximumMoment_(maximumMomentNewtonMeters) {
    if (!std::isfinite(proportionalGain_) || proportionalGain_ <= 0.0 ||
        !std::isfinite(derivativeGain_) || derivativeGain_ < 0.0 ||
        !std::isfinite(maximumMoment_) || maximumMoment_ <= 0.0) {
        throw std::invalid_argument("pitch-controller gains and limit must be finite and physically valid");
    }
}

double PitchController::commandMomentNewtonMeters(
    double pitchErrorRadians,
    double pitchRateRadiansPerSecond) const {
    if (!std::isfinite(pitchErrorRadians) || !std::isfinite(pitchRateRadiansPerSecond)) {
        throw std::invalid_argument("pitch error and rate must be finite");
    }
    return std::clamp(
        -proportionalGain_ * pitchErrorRadians - derivativeGain_ * pitchRateRadiansPerSecond,
        -maximumMoment_,
        maximumMoment_);
}
