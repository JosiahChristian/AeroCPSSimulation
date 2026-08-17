#include "FixedWingScenario.hpp"

#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
bool rejects(const std::vector<std::string>& arguments) {
    try {
        static_cast<void>(FixedWingScenario::fromArguments(arguments));
        return false;
    } catch (const std::invalid_argument&) {
        return true;
    }
}
}

int main() {
    const FixedWingScenario defaults = FixedWingScenario::fromArguments({});
    assert(defaults.durationSeconds == 10.0);
    assert(defaults.timeStepSeconds == 0.01);
    assert(defaults.stepCount() == 1000);

    const FixedWingScenario custom = FixedWingScenario::fromArguments(
        {"--duration", "2.5", "--time-step", "0.02", "--output", "flight.csv"});
    assert(custom.durationSeconds == 2.5);
    assert(custom.timeStepSeconds == 0.02);
    assert(custom.outputPath == "flight.csv");
    assert(custom.stepCount() == 125);

    assert(rejects({"--duration"}));
    assert(rejects({"--duration", "0"}));
    assert(rejects({"--duration", "301"}));
    assert(rejects({"--time-step", "0.051"}));
    assert(rejects({"--duration", "0.01", "--time-step", "0.02"}));
    assert(rejects({"--time-step", "nan"}));
    assert(rejects({"--unknown", "1"}));
    assert(rejects({"--output", ""}));
    return 0;
}
