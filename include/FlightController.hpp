#ifndef FLIGHT_CONTROLLER_HPP
#define FLIGHT_CONTROLLER_HPP

/**
 * @brief Proportional-Integral-Derivative (PID) Flight Controller for Cyber-Physical Systems.
 * Models industrial feedback loops used in automated drones and aerospace actuators.
 */
class FlightController {
public:
    /**
     * @brief Construct a new Flight Controller object with specific gains
     * @param kp Proportional gain (handles immediate tracking error)
     * @param ki Integral gain (removes steady-state steady error over time)
     * @param kd Derivative gain (dampens rapid oscillations/predictions)
     */
    FlightController(double kp, double ki, double kd);

    /**
     * @brief Computes the necessary motor thrust output based on altitude tracking error
     * @param target_altitude The desired altitude setpoint (meters)
     * @param current_altitude The current telemetry altitude reading (meters)
     * @param dt The physical time step interval (seconds)
     * @return double Calculated motor thrust force
     */
    double computeThrust(double target_altitude, double current_altitude, double dt);

    // Reset internal state error registers (crucial for safety loops)
    void reset();

private:
    // PID gain coefficients
    double kp_;
    double ki_;
    double kd_;

    // Internal state tracking registers for calculus integration/differentiation
    double integral_error_ = 0.0;
    double previous_error_ = 0.0;
};

#endif // FLIGHT_CONTROLLER_HPP
