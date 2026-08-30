#pragma once

#include "FlightModels.hpp"

enum class ControlGateDecision {
    Accept,
    Veto
};

struct ControlCommandProposal {
    VehicleState currentState;
    double targetAltitude;
    double proposedThrust;
};

class ControlCommandGate {
public:
    virtual ~ControlCommandGate() = default;

    [[nodiscard]] virtual ControlGateDecision evaluate(
        const ControlCommandProposal& proposal) const = 0;

    [[nodiscard]] virtual const char* name() const noexcept = 0;
};
