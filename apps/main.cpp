#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>

// Simple Drone Struct to simulate physical states
struct Quadcopter {
    double altitude = 0.0;     // current height in meters
    double velocity = 0.0;     // vertical speed in m/s
    double target_alt = 50.0;  // target hover height in meters
};

int main() {
    std::cout << "===============================================" << std::endl;
    std::cout << "      ODU CPS AEROSPACE SIMULATOR ENGINE       " << std::endl;
    std::cout << "===============================================" << std::endl;
    std::cout << "Initializing Quadcopter Flight Systems..." << std::endl;

    Quadcopter drone;
    double time_step = 0.1; // 100ms per physics update frame
    int simulation_steps = 40;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\nStarting Launch Sequence. Target Altitude: " << drone.target_alt << "m\n" << std::endl;
    std::cout << "Time(s)  |  Altitude(m)  |  Velocity(m/s)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    // Simulation Loop running a simple proportional control physics model
    for (int step = 0; step <= simulation_steps; ++step) {
        double current_time = step * time_step;

        // Proportional Control: thrust is proportional to error distance
        double error = drone.target_alt - drone.altitude;
        double thrust = error * 0.5;

        // Physics Update (Euler Integration)
        double gravity = -9.81;
        double acceleration = thrust + gravity;
       
        drone.velocity += acceleration * time_step;
        drone.altitude += drone.velocity * time_step;

        // Prevent drone from falling through the ground
        if (drone.altitude < 0.0) {
            drone.altitude = 0.0;
            drone.velocity = 0.0;
        }

        // Output real-time telemetry state
        std::cout << std::setw(7) << current_time << "  |  "
                  << std::setw(11) << drone.altitude << "  |  "
                  << std::setw(13) << drone.velocity << std::endl;

        // Sleep to mimic real-time telemetry streaming
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Simulation Complete. Telemetry Logged Successfully." << std::endl;
    return 0;
}