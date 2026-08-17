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
        0.2, 4.0, 0.03, 0.05, 0.01, -0.5, 0.5
    };
    const auto loads = AerodynamicModel::longitudinalLoads(
        20.0, 0.1, seaLevelDensity, body, coefficients);
    assert(nearlyEqual(loads.forceBodyNewtons.x, -23.52));
    assert(nearlyEqual(loads.forceBodyNewtons.z, 294.0));
    assert(nearlyEqual(loads.momentBodyNewtonMeters.y, -9.8));

    const auto zeroSpeedLoads = AerodynamicModel::longitudinalLoads(
        0.0, 0.1, seaLevelDensity, body, coefficients);
    assert(nearlyEqual(zeroSpeedLoads.forceBodyNewtons.x, 0.0));
    assert(nearlyEqual(zeroSpeedLoads.forceBodyNewtons.z, 0.0));
    assert(nearlyEqual(zeroSpeedLoads.momentBodyNewtonMeters.y, 0.0));

    return 0;
}
