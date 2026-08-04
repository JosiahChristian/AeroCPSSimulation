#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include "FlightController.hpp" // Hooking into your new module!

struct Quadcopter {
    double altitude = 0.0;     
    double velocity = 0.0;     
    double target_alt = 50.0;  
};

int main() {
    std::cout << "===============================================\n"
              << "   MODULAR ODU CPS AEROSPACE SIMULATOR ENGINE  \n"
              << "===============================================\n";

    Quadcopter drone;
    double time_step = 0.1; // 100ms calculation slices
    int simulation_steps = 60;

    // Initialize your PID Controller with tuned gains: Kp=1.5, Ki=0.2, Kd=0.4
    FlightController controller(1.5, 0.2, 0.4);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\nPID Controller Online. Executing Drone Launch Sequence...\n\n"
              << "Time(s)  |  Altitude(m)  |  Velocity(m/s)\n"
              << "----------------------------------------\n";

    for (int step = 0; step <= simulation_steps; ++step) {
        double current_time = step * time_step;

        // Dynamic thrust calculation using our modular C++ class engine
        double thrust = controller.computeThrust(drone.target_alt, drone.altitude, time_step);

        // Core Physics Integration
        double gravity = -9.81;
        double acceleration = thrust + gravity;
        
        drone.velocity += acceleration * time_step;
        drone.altitude += drone.velocity * time_step;

        if (drone.altitude < 0.0) {
            drone.altitude = 0.0;
            drone.velocity = 0.0;
            controller.reset(); // Reset integration if drone hits the ground
        }

        std::cout << std::setw(7) << current_time << "  |  " 
                  << std::setw(11) << drone.altitude << "  |  " 
                  << std::setw(13) << drone.velocity << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    std::cout << "----------------------------------------\n"
              << "Simulation Lifecycle Complete.\n";
    return 0;
}
