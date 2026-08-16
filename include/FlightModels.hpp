#pragma once

struct VehicleState {
    double altitude{0.0};
    double velocity{0.0};
};

class FlightEnvironment {
public:
    explicit FlightEnvironment(double gravity);

    [[nodiscard]] double gravity() const noexcept { return gravity_; }

private:
    double gravity_;
};
