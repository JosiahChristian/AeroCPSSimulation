# AeroCPSSimulation

[![C++ build and test](https://github.com/JosiahChristian/AeroCPSSimulation/actions/workflows/cpp-simulation-test.yml/badge.svg)](https://github.com/JosiahChristian/AeroCPSSimulation/actions/workflows/cpp-simulation-test.yml)

A deterministic C++ flight-control simulation for exercising cyber-physical-system architecture, feedback control, numerical integration, and telemetry generation.

## Current Scope

The current release models one-dimensional vertical flight. A damped proportional controller combines gravity compensation with altitude-error feedback, and the vehicle state is propagated with fixed-step Euler integration.

The operational command-line scenario remains the verified vertical-flight controller. A separate multi-axis dynamics foundation propagates 3D position, velocity, normalized quaternion attitude, and angular velocity under applied force and torque. Rotational acceleration includes diagonal-inertia gyroscopic coupling.

## Capabilities

- Command-line configuration for altitude, gravity, time step, step limit, and output path
- Explicit simulation initialization and lifecycle validation
- Gravity-compensated proportional control with velocity damping
- Deterministic fixed-step state propagation
- Explicit vehicle-state, environment, and numerical-integrator components
- Validated multi-axis rigid-body state, mass, and diagonal-inertia models
- 3D force and body-axis torque propagation
- Singularity-free quaternion attitude propagation and body-to-world vector rotation
- Coupled diagonal-inertia rotational dynamics
- Ground-contact constraint
- Structured CSV telemetry with scenario metadata on every row
- CMake and CTest integration
- Linux and Windows CI validation
- Strict compiler warnings on supported toolchains

## Architecture

```text
Scenario parameters
        |
Vehicle state + environment
        |
Feedback controller
        |
Semi-implicit Euler integrator
        |
Altitude and velocity telemetry
```

The reusable engine coordinates separate vehicle-state, environment, controller, and integration responsibilities. The command-line application runs bounded scenarios, while independent regression executables validate controller behavior, scenario parsing, physical-model constraints, and numerical refinement.

The independent rigid-body module provides a measured expansion path beyond vertical flight. It uses semi-implicit state updates, normalized quaternion attitude, world-frame forces, body-axis torques, diagonal inertia, and the rigid-body gyroscopic term. It intentionally does not yet claim aerodynamic coefficient models or a complete vehicle-specific flight model.

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

Run a custom Mars-gravity scenario:

```bash
./build/run_simulation --target-altitude 75 --gravity -3.711 --time-step 0.05 --maximum-steps 750 --output telemetry-mars.csv
```

Use `--help` to display every supported option. Invalid, incomplete, non-finite, and out-of-range parameters are rejected before the simulation begins.

Multi-configuration generators, including Visual Studio, place the executable under the selected configuration directory, such as `build/Release/run_simulation.exe`.

The application writes machine-readable telemetry to `telemetry.csv` by default, or to the path supplied with `--output`:

```csv
step,time_s,altitude_m,velocity_mps,target_altitude_m,gravity_mps2
1,0.050000,0.100000,2.000000,50.000000,-9.810000
```

## Verification

Every pull request and push to `main` builds and tests the same CMake targets on Ubuntu and Windows. The regression executable verifies:

- zero-state initialization
- rejection of stepping before initialization
- positive initial ascent response
- convergence to the configured altitude tolerance
- rejection of invalid time steps
- default and custom scenario parsing
- rejection of unknown, missing, non-finite, and out-of-range arguments
- exact velocity propagation under constant acceleration
- first-order position-error reduction as the integration time step is halved
- environment-model gravity constraints
- independent force response across three translational axes
- independent torque response across three rotational axes
- mass, diagonal-inertia, state, force, torque, and time-step validation
- quaternion unit-norm preservation and a known 90-degree frame rotation
- asymmetric-inertia torque-free gyroscopic coupling against hand-computed values

## Engineering Roadmap

Development will proceed in measured layers:

1. aerodynamic force and moment models with coefficient validation
2. vehicle-specific geometry and atmosphere configuration
3. performance measurement and integration with AeroCPSTelemetry

## Related Software

[AeroCPSTelemetry](https://github.com/JosiahChristian/AeroCPSTelemetry) provides the separate browser-based visualization surface for aerospace telemetry and flight-state behavior.

## Status

Active engineering project. Vertical-flight control plus quaternion-based, gyroscopically coupled multi-axis rigid-body propagation are implemented and validated. Vehicle-specific aerodynamics remain planned work.
