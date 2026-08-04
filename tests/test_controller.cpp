#include <gtest/gtest.h>
#include "FlightController.hpp"

// Test 1: Verify that when the drone is at the target, proportional thrust drops to zero
TEST(FlightControllerTest, ZeroErrorYieldsZeroProportionalThrust) {
    // Initialized with Kp=1.0, Ki=0.0, Kd=0.0
    FlightController controller(1.0, 0.0, 0.0);
    
    // Target is 10m, Current is 10m (Error is 0.0)
    double thrust = controller.computeThrust(10.0, 10.0, 0.1);
    
    EXPECT_NEAR(thrust, 0.0, 1e-5);
}

// Test 2: Verify that a positive error generates an upward positive thrust
TEST(FlightControllerTest, PositiveErrorGeneratesPositiveThrust) {
    FlightController controller(1.5, 0.0, 0.0);
    
    // Drone is at 0m, wants to go to 50m (Error is +50.0)
    double thrust = controller.computeThrust(50.0, 0.0, 0.1);
    
    EXPECT_GT(thrust, 0.0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
