#include "FixedWingReference.hpp"

#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

int main() {
    const PitchController controller = FixedWingReference::pitchController();
    assert(controller.commandMomentNewtonMeters(0.1, 0.0) < 0.0);
    assert(controller.commandMomentNewtonMeters(-0.1, 0.0) > 0.0);
    assert(controller.commandMomentNewtonMeters(10.0, 0.0) == -2.0);
    assert(controller.commandMomentNewtonMeters(-10.0, 0.0) == 2.0);

    double pitchError = 0.05;
    double pitchRate = 0.0;
    const double pitchInertia = FixedWingReference::rigidBodyProperties().diagonalInertia().y;
    constexpr double timeStep = 0.001;
    constexpr int steps = 5000;
    for (int step = 0; step < steps; ++step) {
        const double moment = controller.commandMomentNewtonMeters(pitchError, pitchRate);
        pitchRate += moment / pitchInertia * timeStep;
        pitchError += pitchRate * timeStep;
    }
    assert(std::abs(pitchError) < 0.001);
    assert(std::abs(pitchRate) < 0.001);

    bool rejectedGain = false;
    try {
        static_cast<void>(PitchController(0.0, 1.0, 1.0));
    } catch (const std::invalid_argument&) {
        rejectedGain = true;
    }
    assert(rejectedGain);

    bool rejectedNonFinite = false;
    try {
        static_cast<void>(controller.commandMomentNewtonMeters(
            std::numeric_limits<double>::infinity(), 0.0));
    } catch (const std::invalid_argument&) {
        rejectedNonFinite = true;
    }
    assert(rejectedNonFinite);

    return 0;
}
