#include "AerodynamicModel.hpp"
#include "FixedWingReference.hpp"
#include "FixedWingScenario.hpp"
#include "FixedWingTelemetryContract.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char* argv[]) {
    std::vector<std::string> arguments(argv + 1, argv + argc);
    if (arguments.size() == 1 && arguments.front() == "--help") {
        std::cout << FixedWingScenario::usage() << '\n';
        return 0;
    }

    FixedWingScenario scenario;
    try {
        scenario = FixedWingScenario::fromArguments(arguments);
    } catch (const std::invalid_argument& error) {
        std::cerr << "configuration error: " << error.what() << '\n'
                  << FixedWingScenario::usage() << '\n';
        return 2;
    }

    std::ofstream telemetry(scenario.outputPath);
    if (!telemetry) {
        std::cerr << "could not open telemetry output: " << scenario.outputPath << '\n';
        return 3;
    }

    RigidBodyState state = FixedWingReference::trimState();
    const AtmosphereModel atmosphere = FixedWingReference::atmosphere();
    telemetry << FixedWingTelemetryContract::csvHeader << '\n'
              << std::fixed << std::setprecision(9);

    const std::size_t steps = scenario.stepCount();
    for (std::size_t step = 1; step <= steps; ++step) {
        FixedWingReference::advanceTrimStep(state, scenario.timeStepSeconds);
        const auto airData = AerodynamicModel::airDataFromWorldState(
            state.velocityMetersPerSecond, state.attitudeBodyToWorld, atmosphere);
        telemetry << FixedWingTelemetryContract::schemaVersion << ',' << step << ','
                  << step * scenario.timeStepSeconds << ','
                  << state.positionMeters.x << ',' << state.positionMeters.y << ','
                  << state.positionMeters.z << ',' << state.velocityMetersPerSecond.x << ','
                  << state.velocityMetersPerSecond.y << ',' << state.velocityMetersPerSecond.z << ','
                  << state.attitudeBodyToWorld.w << ',' << state.attitudeBodyToWorld.x << ','
                  << state.attitudeBodyToWorld.y << ',' << state.attitudeBodyToWorld.z << ','
                  << state.angularVelocityRadiansPerSecond.x << ','
                  << state.angularVelocityRadiansPerSecond.y << ','
                  << state.angularVelocityRadiansPerSecond.z << ','
                  << airData.airspeedMetersPerSecond << ',' << airData.angleOfAttackRadians << ','
                  << airData.sideslipAngleRadians << '\n';
    }

    std::cout << "fixed-wing trim simulation completed " << steps
              << " steps; telemetry: " << scenario.outputPath << '\n';
    return 0;
}
