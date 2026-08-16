#include "SimulationScenario.hpp"
#include "SimulatorEngine.hpp"
#include "TelemetryContract.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char* argv[]) {
    std::vector<std::string> arguments(argv + 1, argv + argc);
    if (arguments.size() == 1 && arguments.front() == "--help") {
        std::cout << SimulationScenario::usage() << '\n';
        return 0;
    }

    SimulationScenario scenario;
    try {
        scenario = SimulationScenario::fromArguments(arguments);
    } catch (const std::invalid_argument& error) {
        std::cerr << "configuration error: " << error.what() << '\n'
                  << SimulationScenario::usage() << '\n';
        return 2;
    }

    std::ofstream telemetry(scenario.outputPath);
    if (!telemetry) {
        std::cerr << "could not open telemetry output: " << scenario.outputPath << '\n';
        return 3;
    }

    SimulatorEngine simulation(scenario.targetAltitude, scenario.gravity);
    simulation.initializeSystem();

    telemetry << TelemetryContract::csvHeader << '\n'
              << std::fixed << std::setprecision(6);
    std::size_t step = 0;
    for (; step < scenario.maximumSteps && !simulation.isTrajectoryTrackingComplete(); ++step) {
        simulation.executeTimeSliceStep(scenario.timeStep);
        telemetry << TelemetryContract::schemaVersion << ','
                  << step + 1 << ','
                  << (step + 1) * scenario.timeStep << ','
                  << simulation.getCurrentAltitude() << ','
                  << simulation.getCurrentVelocity() << ','
                  << scenario.targetAltitude << ','
                  << scenario.gravity << '\n';
    }

    if (!simulation.isTrajectoryTrackingComplete()) {
        std::cerr << "simulation did not converge within the configured step limit\n";
        return 1;
    }

    std::cout << "simulation converged in " << step << " steps; telemetry: "
              << scenario.outputPath << '\n';
    return 0;
}
