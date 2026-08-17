#pragma once

class PitchController {
public:
    PitchController(
        double proportionalGainNewtonMetersPerRadian,
        double derivativeGainNewtonMeterSecondsPerRadian,
        double maximumMomentNewtonMeters);

    [[nodiscard]] double commandMomentNewtonMeters(
        double pitchErrorRadians,
        double pitchRateRadiansPerSecond) const;

private:
    double proportionalGain_;
    double derivativeGain_;
    double maximumMoment_;
};
