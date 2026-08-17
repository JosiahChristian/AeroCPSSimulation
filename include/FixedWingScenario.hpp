#pragma once

#include <cstddef>
#include <string>
#include <vector>

struct FixedWingScenario {
    double durationSeconds{10.0};
    double timeStepSeconds{0.01};
    std::string outputPath{"fixed-wing-telemetry.csv"};

    [[nodiscard]] std::size_t stepCount() const;
    [[nodiscard]] static FixedWingScenario fromArguments(const std::vector<std::string>& arguments);
    [[nodiscard]] static std::string usage();
};
