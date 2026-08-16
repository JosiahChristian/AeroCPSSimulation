#include "SimulatorEngine.hpp"

#include <iomanip>
#include <iostream>

int main() {
    constexpr double targetAltitude = 50.0;
    constexpr double earthGravity = -9.81;
    constexpr double timeStep = 0.05;
    constexpr int maximumSteps = 500;

    SimulatorEngine simulation(targetAltitude, earthGravity);
    simulation.initializeSystem();

    std::cout << "time_s,altitude_m,velocity_mps\n" << std::fixed << std::setprecision(3);
    int step = 0;
    for (; step < maximumSteps && !simulation.isTrajectoryTrackingComplete(); ++step) {
        simulation.executeTimeSliceStep(timeStep);
        std::cout << (step + 1) * timeStep << ','
                  << simulation.getCurrentAltitude() << ','
                  << simulation.getCurrentVelocity() << '\n';
    }

    if (!simulation.isTrajectoryTrackingComplete()) {
        std::cerr << "simulation did not converge within the configured step limit\n";
        return 1;
    }

    return 0;
}
