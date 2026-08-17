#include "FixedWingScenario.hpp"

#include <cmath>
#include <stdexcept>

namespace {
constexpr double maximumDurationSeconds = 300.0;
constexpr double maximumTimeStepSeconds = 0.05;
constexpr std::size_t maximumStepCount = 1'000'000;

double parsePositiveDouble(const std::string& option, const std::string& value) {
    std::size_t consumed = 0;
    double result = 0.0;
    try {
        result = std::stod(value, &consumed);
    } catch (const std::exception&) {
        throw std::invalid_argument(option + " requires a numeric value");
    }
    if (consumed != value.size() || !std::isfinite(result) || result <= 0.0) {
        throw std::invalid_argument(option + " requires a finite positive value");
    }
    return result;
}
}

std::size_t FixedWingScenario::stepCount() const {
    const double count = std::floor(durationSeconds / timeStepSeconds + 1e-12);
    if (!std::isfinite(count) || count < 1.0) {
        throw std::invalid_argument("--duration must include at least one complete time step");
    }
    if (count > static_cast<double>(maximumStepCount)) {
        throw std::invalid_argument("scenario exceeds the maximum step count");
    }
    return static_cast<std::size_t>(count);
}

FixedWingScenario FixedWingScenario::fromArguments(const std::vector<std::string>& arguments) {
    FixedWingScenario scenario;
    for (std::size_t index = 0; index < arguments.size(); index += 2) {
        const std::string& option = arguments[index];
        if (index + 1 >= arguments.size()) {
            throw std::invalid_argument(option + " requires a value");
        }
        const std::string& value = arguments[index + 1];
        if (option == "--duration") {
            scenario.durationSeconds = parsePositiveDouble(option, value);
        } else if (option == "--time-step") {
            scenario.timeStepSeconds = parsePositiveDouble(option, value);
        } else if (option == "--output") {
            if (value.empty()) {
                throw std::invalid_argument("--output requires a non-empty path");
            }
            scenario.outputPath = value;
        } else {
            throw std::invalid_argument("unknown option: " + option);
        }
    }

    if (scenario.durationSeconds > maximumDurationSeconds) {
        throw std::invalid_argument("--duration must not exceed 300 seconds");
    }
    if (scenario.timeStepSeconds > maximumTimeStepSeconds) {
        throw std::invalid_argument("--time-step must not exceed 0.05 seconds");
    }
    static_cast<void>(scenario.stepCount());
    return scenario;
}

std::string FixedWingScenario::usage() {
    return "Usage: run_fixed_wing [--duration SECONDS] [--time-step SECONDS] [--output PATH]";
}
