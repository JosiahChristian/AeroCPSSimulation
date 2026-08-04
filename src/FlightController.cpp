#include "../include/FlightController.hpp"

FlightController::FlightController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), integral_error_(0.0), previous_error_(0.0) {}

double FlightController::computeThrust(double target_altitude, double current_altitude, double dt) {
    // 1. Calculate Proportional error (Distance to target)
    double error = target_altitude - current_altitude;

    // 2. Calculate Integral error (Accumulating past error to fight gravity sag)
    integral_error_ += error * dt;

    // 3. Calculate Derivative error (Rate of change to prevent overshoot)
    double derivative_error = (error - previous_error_) / dt;

    // 4. Combine into final control command (PID Law)
    double control_output = (kp_ * error) + (ki_ * integral_error_) + (kd_ * derivative_error);

    // Save state register for the next step calculation
    previous_error_ = error;

    return control_output;
}

void FlightController::reset() {
    integral_error_ = 0.0;
    previous_error_ = 0.0;
}
