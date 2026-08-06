#ifndef FLIGHT_CONTROLLER_HPP
#define FLIGHT_CONTROLLER_HPP

class FlightController {
public:
    FlightController(double targetAltitude);
    void updateState(double timeStep);
    double getAltitude() const;
    double getVelocity() const;

private:
    double targetAlt;
    double currentAlt;
    double velocity;
};

#endif
