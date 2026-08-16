#include "RigidBodyDynamics.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>

int main() {
    constexpr std::size_t steps = 250000;
    constexpr double timeStep = 0.001;
    const RigidBodyProperties properties(12.0, {2.0, 3.0, 4.0});
    RigidBodyState state;

    const auto start = std::chrono::steady_clock::now();
    for (std::size_t step = 0; step < steps; ++step) {
        RigidBodyDynamics::advance(
            state,
            properties,
            {0.1, -0.05, 0.02},
            {0.001, -0.002, 0.0015},
            timeStep);
    }
    const auto finish = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(finish - start).count();

    if (!state.positionMeters.isFinite() || !state.attitudeBodyToWorld.isFinite() ||
        !std::isfinite(seconds) || seconds <= 0.0) {
        std::cerr << "performance smoke run produced invalid state or timing\n";
        return 1;
    }

    std::cout << "steps=" << steps
              << " elapsed_seconds=" << seconds
              << " steps_per_second=" << static_cast<double>(steps) / seconds << '\n';
    return 0;
}
