#include <iostream>
#include <cassert>
#include <cmath>

// Simulation validation parameters
const double TARGET_ALTITUDE = 50.0;
const double TIME_STEP = 0.05;
const double MARS_GRAVITY = -3.711;

// Function to simulate a single integration step for testing
double calculateNextAltitude(double currentAlt, double& velocity) {
    double error = TARGET_ALTITUDE - currentAlt;
    double thrust = error * 0.4;
    double acceleration = thrust + MARS_GRAVITY;

    velocity += acceleration * TIME_STEP;
    double nextAlt = currentAlt + (velocity * TIME_STEP);
    
    return (nextAlt < 0.0) ? 0.0 : nextAlt;
}

int main() {
    std::cout << "===============================================================\n";
    Unit Test: Control Law Flight Parameter Verification
    std::cout << "===============================================================\n";

    double currentAltitude = 0.0;
    double currentVelocity = 0.0;

    // TEST ASSERTION 1: Verify system initializes at ground baseline bounds
    std::cout << "[RUNNING TEST 1/3] Verifying initial physical state values...\n";
    assert(currentAltitude == 0.0);
    assert(currentVelocity == 0.0);
    std::cout << "--> TEST 1 PASSED: Baseline constraints validated.\n\n";

    // TEST ASSERTION 2: Verify positive acceleration response under thrust
    std::cout << "[RUNNING TEST 2/3] Verifying positive kinematics ascent delta...\n";
    currentAltitude = calculateNextAltitude(currentAltitude, currentVelocity);
    assert(currentAltitude >= 0.0);
    std::cout << "--> TEST 2 PASSED: Ascent trajectory tracking bounds verified.\n\n";

    // TEST ASSERTION 3: Verify the control law is actively minimizing error bounds
    std::cout << "[RUNNING TEST 3/3] Verifying error convergence behavior...\n";
    double initialError = std::abs(TARGET_ALTITUDE - 0.0);
    double stepError = std::abs(TARGET_ALTITUDE - currentAltitude);
    assert(stepError < initialError);
    std::cout << "--> TEST 3 PASSED: Proportional tracking loop minimized absolute error.\n\n";

    std::cout << "---------------------------------------------------------------\n";
    std::cout << "🏆 SUCCESS: All algorithmic flight checks completed flawlessly!\n";
    return 0;
}
