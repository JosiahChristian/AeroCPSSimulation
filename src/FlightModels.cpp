#include "FlightModels.hpp"

#include <cmath>
#include <stdexcept>

FlightEnvironment::FlightEnvironment(double gravity) : gravity_(gravity) {
    if (!std::isfinite(gravity_) || gravity_ >= 0.0) {
        throw std::invalid_argument("environment gravity must be finite and negative");
    }
}
