# AeroCPSSimulation

[![C++ build and test](https://github.com/JosiahChristian/AeroCPSSimulation/actions/workflows/cpp-simulation-test.yml/badge.svg)](https://github.com/JosiahChristian/AeroCPSSimulation/actions/workflows/cpp-simulation-test.yml)

A deterministic C++ flight-control simulation for exercising cyber-physical-system architecture, feedback control, numerical integration, and telemetry generation.

## Current Scope

The current release models one-dimensional vertical flight. A damped proportional controller combines gravity compensation with altitude-error feedback, and the vehicle state is propagated with fixed-step Euler integration.

This focused model provides a verified software foundation for later multi-axis dynamics without presenting planned capabilities as already implemented.

## Capabilities

- Configurable target altitude and planetary gravity
- Explicit simulation initialization and lifecycle validation
- Gravity-compensated proportional control with velocity damping
- Deterministic fixed-step state propagation
- Ground-contact constraint
- CSV telemetry written to standard output
- CMake and CTest integration
- Linux and Windows CI validation
- Strict compiler warnings on supported toolchains

## Architecture

```text
Scenario parameters
        |
SimulatorEngine
        |
Feedback controller
        |
State integration
        |
Altitude and velocity telemetry
```

The reusable engine lives in `src/SimulatorEngine.cpp` with its public interface in `include/SimulatorEngine.hpp`. The command-line application in `apps/main.cpp` runs a bounded scenario, while `tests/test_controller.cpp` validates lifecycle rules, state evolution, convergence, and invalid-input handling.

## Build and Run

Requirements:

- CMake 3.22 or newer
- A C++17 compiler

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Run the simulator:

```bash
./build/run_simulation
```

Multi-configuration generators, including Visual Studio, place the executable under the selected configuration directory, such as `build/Release/run_simulation.exe`.

The application emits machine-readable telemetry:

```csv
time_s,altitude_m,velocity_mps
0.050,0.100,2.000
0.100,0.294,3.860
```

## Verification

Every pull request and push to `main` builds and tests the same CMake targets on Ubuntu and Windows. The regression executable verifies:

- zero-state initialization
- rejection of stepping before initialization
- positive initial ascent response
- convergence to the configured altitude tolerance
- rejection of invalid time steps

## Engineering Roadmap

Development will proceed in measured layers:

1. configurable scenario input and structured telemetry files
2. additional numerical-integrator verification
3. explicit vehicle and environment models
4. multi-axis rigid-body state and dynamics
5. performance measurement and integration with AeroCPSTelemetry

## Related Software

[AeroCPSTelemetry](https://github.com/JosiahChristian/AeroCPSTelemetry) provides the separate browser-based visualization surface for aerospace telemetry and flight-state behavior.

## Status

Active engineering project. The vertical-flight simulation foundation is implemented and validated; multi-axis flight dynamics remain planned work.
