#pragma once

#include <cstddef>
#include <string>
#include <vector>

struct SimulationScenario {
    double targetAltitude{50.0};
    double gravity{-9.81};
    double timeStep{0.05};
    std::size_t maximumSteps{500};
    std::string outputPath{"telemetry.csv"};

    [[nodiscard]] static SimulationScenario fromArguments(const std::vector<std::string>& arguments);
    [[nodiscard]] static std::string usage();
};
