#include "SimulationScenario.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
template <typename Callable>
bool rejects(Callable callable) {
    try {
        callable();
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}
}

int main() {
    const SimulationScenario defaults = SimulationScenario::fromArguments({});
    assert(std::abs(defaults.targetAltitude - 50.0) < 1e-9);
    assert(std::abs(defaults.gravity + 9.81) < 1e-9);
    assert(defaults.maximumSteps == 500);
    assert(defaults.outputPath == "telemetry.csv");

    const SimulationScenario custom = SimulationScenario::fromArguments({
        "--target-altitude", "125.5",
        "--gravity", "-3.711",
        "--time-step", "0.01",
        "--maximum-steps", "2000",
        "--output", "mars.csv"
    });
    assert(std::abs(custom.targetAltitude - 125.5) < 1e-9);
    assert(std::abs(custom.gravity + 3.711) < 1e-9);
    assert(std::abs(custom.timeStep - 0.01) < 1e-9);
    assert(custom.maximumSteps == 2000);
    assert(custom.outputPath == "mars.csv");

    assert(rejects([] { static_cast<void>(SimulationScenario::fromArguments({"--unknown", "1"})); }));
    assert(rejects([] { static_cast<void>(SimulationScenario::fromArguments({"--gravity"})); }));
    assert(rejects([] { static_cast<void>(SimulationScenario::fromArguments({"--gravity", "9.81"})); }));
    assert(rejects([] { static_cast<void>(SimulationScenario::fromArguments({"--time-step", "0"})); }));
    assert(rejects([] { static_cast<void>(SimulationScenario::fromArguments({"--maximum-steps", "0"})); }));
    assert(rejects([] { static_cast<void>(SimulationScenario::fromArguments({"--target-altitude", "nan"})); }));

    assert(SimulationScenario::usage().find("--target-altitude") != std::string::npos);
    return 0;
}
