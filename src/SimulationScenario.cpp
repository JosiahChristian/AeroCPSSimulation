#include "SimulationScenario.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
double parseDouble(const std::string& option, const std::string& value) {
    std::size_t consumed = 0;
    double result = 0.0;
    try {
        result = std::stod(value, &consumed);
    } catch (const std::exception&) {
        throw std::invalid_argument(option + " requires a numeric value");
    }
    if (consumed != value.size() || !std::isfinite(result)) {
        throw std::invalid_argument(option + " requires a finite numeric value");
    }
    return result;
}

std::size_t parsePositiveSize(const std::string& option, const std::string& value) {
    std::size_t consumed = 0;
    unsigned long long result = 0;
    try {
        result = std::stoull(value, &consumed);
    } catch (const std::exception&) {
        throw std::invalid_argument(option + " requires a positive integer");
    }
    if (consumed != value.size() || result == 0 || result > std::numeric_limits<std::size_t>::max()) {
        throw std::invalid_argument(option + " requires a positive integer in range");
    }
    return static_cast<std::size_t>(result);
}
}

SimulationScenario SimulationScenario::fromArguments(const std::vector<std::string>& arguments) {
    SimulationScenario scenario;

    for (std::size_t index = 0; index < arguments.size(); index += 2) {
        const std::string& option = arguments[index];
        if (index + 1 >= arguments.size()) {
            throw std::invalid_argument(option + " requires a value");
        }
        const std::string& value = arguments[index + 1];

        if (option == "--target-altitude") {
            scenario.targetAltitude = parseDouble(option, value);
        } else if (option == "--gravity") {
            scenario.gravity = parseDouble(option, value);
        } else if (option == "--time-step") {
            scenario.timeStep = parseDouble(option, value);
        } else if (option == "--maximum-steps") {
            scenario.maximumSteps = parsePositiveSize(option, value);
        } else if (option == "--output") {
            if (value.empty()) {
                throw std::invalid_argument("--output requires a non-empty path");
            }
            scenario.outputPath = value;
        } else {
            throw std::invalid_argument("unknown option: " + option);
        }
    }

    if (scenario.targetAltitude < 0.0) {
        throw std::invalid_argument("--target-altitude must be non-negative");
    }
    if (scenario.gravity >= 0.0) {
        throw std::invalid_argument("--gravity must be negative");
    }
    if (scenario.timeStep <= 0.0) {
        throw std::invalid_argument("--time-step must be positive");
    }

    return scenario;
}

std::string SimulationScenario::usage() {
    return "Usage: run_simulation [--target-altitude METERS] [--gravity MPS2] "
           "[--time-step SECONDS] [--maximum-steps COUNT] [--output PATH]";
}
