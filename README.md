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
- Configurable exponential atmosphere density and steady world-frame wind
- Quadratic drag with validated reference area and drag coefficient
- Configurable linear lift, induced drag, and pitching-moment coefficient model
- Configurable sideslip, roll-rate, and yaw-rate lateral-directional derivatives
- Combined six-axis aerodynamic load assembly and body-to-world force application
- Wind-relative body-frame airspeed, angle-of-attack, and sideslip derivation
- Representative 8 kg small fixed-wing reference configuration with verified level-flight trim
- Ground-contact constraint
- Structured CSV telemetry with scenario metadata on every row
- Versioned `aerocps.telemetry.v1` contract for downstream consumers
- Cross-platform performance smoke executable
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

The independent rigid-body module provides a measured expansion path beyond vertical flight. It uses semi-implicit state updates, normalized quaternion attitude, world-frame forces, body-axis torques, diagonal inertia, and the rigid-body gyroscopic term. The aerodynamic module derives wind-relative body velocity, airspeed, angle of attack, and sideslip from world state; it then evaluates configurable longitudinal and lateral-directional coefficients. Combined aerodynamic loads can be applied in body axes; force is rotated into the world frame through the current attitude while torque remains in body axes.

The included `FixedWingReference` is a representative engineering fixture, not a certified or identified aircraft model. Its mass, diagonal inertia, geometry, coefficients, and thrust are internally constructed around an exact 20 m/s straight-and-level trim condition at 100 m altitude. This gives the six-degree-of-freedom path a reproducible integration scenario without overstating real-aircraft fidelity.

## Build and Run

Requirements:

- CMake 3.22 or newer
- A C++17 compiler
- Ninja when using the checked-in CMake presets

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

For a reproducible Ninja build, use `cmake --preset default`,
`cmake --build --preset default`, and `ctest --preset default`. A separate
`sanitizers` preset enables AddressSanitizer and UndefinedBehaviorSanitizer on
GCC and Clang; CI runs that preset on every push and pull request.

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
schema_version,step,time_s,altitude_m,velocity_mps,target_altitude_m,gravity_mps2
aerocps.telemetry.v1,1,0.050000,0.100000,2.000000,50.000000,-9.810000
```

The field definitions, units, and compatibility policy are documented in [`docs/telemetry-contract.md`](docs/telemetry-contract.md) for integration with AeroCPSTelemetry and other consumers.

## Verification

Every pull request and push to `main` builds and tests the same CMake targets on Ubuntu and Windows. A separate Linux lane runs the complete suite with address and undefined-behavior sanitizers. The regression executable verifies:

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
- exponential density at sea level and one scale height
- quadratic drag magnitude and direction against hand-computed values
- wind-relative drag reduction and zero-relative-airspeed behavior
- exact telemetry schema version and required field contract
- finite-state 250,000-step rigid-body performance smoke run
- hand-calculated lift, induced-drag, and pitching-moment coefficient case
- hand-calculated side-force, rolling-moment, and yawing-moment derivative case
- exact six-axis load composition and a known 90-degree body-force frame transformation
- wind-relative air-data derivation and body/world quaternion round-trip cases
- 10-second fixed-wing trim propagation with bounded position, velocity, attitude, and angular-rate residuals

## Engineering Roadmap

Development will proceed in measured layers:

1. perturbation-recovery control for the fixed-wing reference
2. command-line six-degree-of-freedom scenario selection and telemetry
3. direct ingestion integration with AeroCPSTelemetry

## Related Software

[AeroCPSTelemetry](https://github.com/JosiahChristian/AeroCPSTelemetry) provides the separate browser-based visualization surface for aerospace telemetry and flight-state behavior.

## Status

Active engineering project. Vertical-flight control, quaternion-based coupled rigid-body propagation, wind-relative air-data derivation, configurable six-axis aerodynamic loads, frame-aware load application, and a representative trimmed small fixed-wing integration fixture are implemented and validated. Real-aircraft identification and validation remain outside the current scope.
