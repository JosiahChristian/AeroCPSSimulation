# AeroCPSSimulation

A modular C++ flight-dynamics and feedback-control simulation for cyber-physical systems.

## Overview

AeroCPSSimulation models aircraft state evolution and closed-loop control in a native C++ simulation environment. The codebase separates simulation state, vehicle dynamics, environmental parameters, and control behavior so individual components can be inspected and extended independently.

The project emphasizes deterministic execution, reproducible native builds, numerical simulation, and testable systems code.

## Core Capabilities

- Six-degree-of-freedom state representation
- Configurable simulation environment
- Feedback-control behavior
- Numerical state propagation
- Modular C++ components
- CMake-based builds
- Automated validation

## Architecture

Environment / Configuration
          |
      Vehicle State
          |
     Flight Dynamics
          |
  Numerical Integration
          |
      Controller
          |
     Updated State

## Technology

**Language:** C++  
**Build:** CMake  
**Domain:** Flight dynamics, feedback control, numerical simulation, cyber-physical systems  
**Engineering:** Modular native code, reproducible builds, automated validation

## Build

    cmake -S . -B build
    cmake --build build

Run the generated simulation executable from the build directory appropriate to your platform and generator.

## Engineering Goals

The project remains intentionally focused as a simulation engine rather than a general-purpose flight simulator. Continued development prioritizes stronger verification, explicit numerical assumptions, configurable scenarios, structured telemetry output, and measured performance before increasing model complexity.

## Related Software

**AeroCPSTelemetry** provides a separate browser-based visualization surface for aerospace telemetry and flight-state behavior.