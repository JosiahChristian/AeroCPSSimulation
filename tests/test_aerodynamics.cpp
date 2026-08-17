#include "AerodynamicModel.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>

namespace {
bool nearlyEqual(double left, double right, double tolerance = 1e-10) {
    return std::abs(left - right) <= tolerance;
}
}

int main() {
    constexpr double seaLevelDensity = 1.225;
    constexpr double scaleHeight = 8500.0;
    const AtmosphereModel stillAir(seaLevelDensity, scaleHeight);
    const AerodynamicProperties body(2.0, 0.5);

    assert(nearlyEqual(stillAir.densityAtAltitude(0.0), seaLevelDensity));
    assert(nearlyEqual(stillAir.densityAtAltitude(scaleHeight), seaLevelDensity / std::exp(1.0)));
    assert(stillAir.densityAtAltitude(10000.0) < stillAir.densityAtAltitude(1000.0));

    const Vector3 drag = AerodynamicModel::dragForceWorld({10.0, 0.0, 0.0}, 0.0, stillAir, body);
    assert(nearlyEqual(drag.x, -61.25));
    assert(nearlyEqual(drag.y, 0.0));
    assert(nearlyEqual(drag.z, 0.0));

    const Vector3 stationary = AerodynamicModel::dragForceWorld({}, 0.0, stillAir, body);
    assert(nearlyEqual(stationary.x, 0.0));
    assert(nearlyEqual(stationary.y, 0.0));
    assert(nearlyEqual(stationary.z, 0.0));

    const AtmosphereModel tailwind(seaLevelDensity, scaleHeight, {5.0, 0.0, 0.0});
    const Vector3 windDrag = AerodynamicModel::dragForceWorld({10.0, 0.0, 0.0}, 0.0, tailwind, body);
    assert(nearlyEqual(windDrag.x, -15.3125));

    bool rejectedAltitude = false;
    try {
        static_cast<void>(stillAir.densityAtAltitude(-1.0));
    } catch (const std::invalid_argument&) {
        rejectedAltitude = true;
    }
    assert(rejectedAltitude);

    bool rejectedArea = false;
    try {
        static_cast<void>(AerodynamicProperties(0.0, 0.5));
    } catch (const std::invalid_argument&) {
        rejectedArea = true;
    }
    assert(rejectedArea);

    const AerodynamicModel::LongitudinalCoefficients coefficients{
        0.2, 4.0, 0.03, 0.05, 0.01, -0.5, -8.0, 0.5, 0.35, 1.4
    };
    const auto loads = AerodynamicModel::longitudinalLoads(
        20.0, 0.1, 0.0, seaLevelDensity, body, coefficients);
    assert(nearlyEqual(loads.forceBodyNewtons.x, -23.52));
    assert(nearlyEqual(loads.forceBodyNewtons.z, 294.0));
    assert(nearlyEqual(loads.momentBodyNewtonMeters.y, -9.8));

    const auto zeroSpeedLoads = AerodynamicModel::longitudinalLoads(
        0.0, 0.1, 0.4, seaLevelDensity, body, coefficients);
    assert(nearlyEqual(zeroSpeedLoads.forceBodyNewtons.x, 0.0));
    assert(nearlyEqual(zeroSpeedLoads.forceBodyNewtons.z, 0.0));
    assert(nearlyEqual(zeroSpeedLoads.momentBodyNewtonMeters.y, 0.0));

    const auto highAngleLoads = AerodynamicModel::longitudinalLoads(
        20.0, 10.0, 0.0, seaLevelDensity, body, coefficients);
    const auto negativeHighAngleLoads = AerodynamicModel::longitudinalLoads(
        20.0, -10.0, 0.0, seaLevelDensity, body, coefficients);
    assert(nearlyEqual(highAngleLoads.forceBodyNewtons.z, 686.0));
    assert(nearlyEqual(negativeHighAngleLoads.forceBodyNewtons.z, -588.0));
    assert(std::isfinite(highAngleLoads.forceBodyNewtons.x));
    assert(std::isfinite(highAngleLoads.momentBodyNewtonMeters.y));

    bool rejectedEnvelope = false;
    try {
        auto invalidCoefficients = coefficients;
        invalidCoefficients.maximumLinearAngleOfAttackRadians = 0.0;
        static_cast<void>(AerodynamicModel::longitudinalLoads(
            20.0, 0.1, 0.0, seaLevelDensity, body, invalidCoefficients));
    } catch (const std::invalid_argument&) {
        rejectedEnvelope = true;
    }
    assert(rejectedEnvelope);

    const auto pitchRateLoads = AerodynamicModel::longitudinalLoads(
        20.0, 0.1, 0.4, seaLevelDensity, body, coefficients);
    assert(nearlyEqual(pitchRateLoads.momentBodyNewtonMeters.y, -19.6));

    const AerodynamicModel::LateralDirectionalCoefficients lateralCoefficients{
        -0.8, 0.1, 0.2,
        -0.2, -0.5, 0.1,
        0.3, -0.05, -0.2,
        2.0
    };
    const auto lateralLoads = AerodynamicModel::lateralDirectionalLoads(
        20.0, 0.1, 0.4, -0.2, seaLevelDensity, body, lateralCoefficients);
    assert(nearlyEqual(lateralLoads.forceBodyNewtons.y, -39.2));
    assert(nearlyEqual(lateralLoads.momentBodyNewtonMeters.x, -30.38));
    assert(nearlyEqual(lateralLoads.momentBodyNewtonMeters.z, 30.38));

    const auto zeroSpeedLateralLoads = AerodynamicModel::lateralDirectionalLoads(
        0.0, 0.1, 0.4, -0.2, seaLevelDensity, body, lateralCoefficients);
    assert(nearlyEqual(zeroSpeedLateralLoads.forceBodyNewtons.y, 0.0));
    assert(nearlyEqual(zeroSpeedLateralLoads.momentBodyNewtonMeters.x, 0.0));
    assert(nearlyEqual(zeroSpeedLateralLoads.momentBodyNewtonMeters.z, 0.0));

    bool rejectedSpan = false;
    try {
        auto invalidCoefficients = lateralCoefficients;
        invalidCoefficients.referenceSpanMeters = 0.0;
        static_cast<void>(AerodynamicModel::lateralDirectionalLoads(
            20.0, 0.1, 0.4, -0.2, seaLevelDensity, body, invalidCoefficients));
    } catch (const std::invalid_argument&) {
        rejectedSpan = true;
    }
    assert(rejectedSpan);

    const auto combinedLoads = AerodynamicModel::combinedLoads(
        20.0, 0.1, 0.1, {0.4, 0.0, -0.2}, seaLevelDensity, body,
        coefficients, lateralCoefficients);
    assert(nearlyEqual(combinedLoads.forceBodyNewtons.x, -23.52));
    assert(nearlyEqual(combinedLoads.forceBodyNewtons.y, -39.2));
    assert(nearlyEqual(combinedLoads.forceBodyNewtons.z, 294.0));
    assert(nearlyEqual(combinedLoads.momentBodyNewtonMeters.x, -30.38));
    assert(nearlyEqual(combinedLoads.momentBodyNewtonMeters.y, -9.8));
    assert(nearlyEqual(combinedLoads.momentBodyNewtonMeters.z, 30.38));

    const auto airData = AerodynamicModel::airDataFromWorldState(
        {15.0, 2.0, 1.0}, Quaternion{}, tailwind);
    assert(nearlyEqual(airData.velocityBodyMetersPerSecond.x, 10.0));
    assert(nearlyEqual(airData.velocityBodyMetersPerSecond.y, 2.0));
    assert(nearlyEqual(airData.velocityBodyMetersPerSecond.z, 1.0));
    assert(nearlyEqual(airData.airspeedMetersPerSecond, std::sqrt(105.0)));
    assert(nearlyEqual(airData.angleOfAttackRadians, std::atan2(1.0, 10.0)));
    assert(nearlyEqual(airData.sideslipAngleRadians, std::asin(2.0 / std::sqrt(105.0))));

    const Quaternion quarterTurn = Quaternion::fromAxisAngle(
        {0.0, 0.0, 1.0}, std::acos(-1.0) / 2.0);
    const auto rotatedAirData = AerodynamicModel::airDataFromWorldState(
        {0.0, 10.0, 0.0}, quarterTurn, stillAir);
    assert(nearlyEqual(rotatedAirData.velocityBodyMetersPerSecond.x, 10.0));
    assert(nearlyEqual(rotatedAirData.velocityBodyMetersPerSecond.y, 0.0));

    const auto stateLoads = AerodynamicModel::loadsFromWorldState(
        {20.0, 0.0, 0.0}, 0.0, Quaternion{}, {}, stillAir, body,
        coefficients, lateralCoefficients);
    const auto expectedStateLoads = AerodynamicModel::combinedLoads(
        20.0, 0.0, 0.0, {}, seaLevelDensity, body,
        coefficients, lateralCoefficients);
    assert(nearlyEqual(stateLoads.forceBodyNewtons.x, expectedStateLoads.forceBodyNewtons.x));
    assert(nearlyEqual(stateLoads.forceBodyNewtons.y, expectedStateLoads.forceBodyNewtons.y));
    assert(nearlyEqual(stateLoads.forceBodyNewtons.z, expectedStateLoads.forceBodyNewtons.z));

    return 0;
}
