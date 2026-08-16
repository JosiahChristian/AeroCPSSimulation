#pragma once

#include "FlightModels.hpp"

class NumericalIntegrator {
public:
    static void advanceSemiImplicitEuler(VehicleState& state, double acceleration, double timeStep);
};
