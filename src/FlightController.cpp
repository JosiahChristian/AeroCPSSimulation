#include "../include/FlightController.hpp"
#include <iostream>

FlightController::FlightController(double targetAltitude) {
    targetAlt = targetAltitude;
    currentAlt = 0.0;
    velocity = 0.0;
}

void FlightController::updateState(double timeStep) {
    // Proportional feedback control law loop equations
    // Change gravity from -9.81 to -3.711 here later to run your Mars test!
    double error = targetAlt - currentAlt;
    double thrust = error * 0.5;
    double gravity = -3.711; 
    double acceleration = thrust + gravity;

    velocity += acceleration * timeStep;
    currentAlt += velocity * timeStep;

    if (currentAlt < 0.0) {
        currentAlt = 0.0;
        velocity = 0.0;
    }
}

double FlightController::getAltitude() const { return currentAlt; }
double FlightController::getVelocity() const { return velocity; }
