# AeroCPSSimulation: 6-DOF Drone Control Engine

A high-fidelity Cyber-Physical System (CPS) simulation engine modeling quadcopter flight dynamics. This project serves as a foundational platform for evaluating Proportional-Integral-Derivative (PID) flight control laws and digital-twin telemetry architectures.

## 📊 Core Physics & System Dynamics

The simulator models vertical flight dynamics using basic Euler integration. The thrust command relies on a simple proportional tracking loop calculated from the system's altitude error state:

$$e(t) = z_{target} - z(t)$$

$$a(t) = \frac{T(t) + F_g}{m}$$

Where:
* $z(t)$ = Current Altitude (m)
* $e(t)$ = Tracking Error (m)
* $T(t)$ = Controller Motor Thrust ($e \times 0.5$)
* $F_g$ = Acceleration due to gravity ($-9.81 m/s^2$)

## 🛠️ Repository Architecture

* `apps/`: Entry application executables running active runtime simulation loops.
* `include/`: C++ header definitions for control laws and environment variables.
* `src/`: Core physics algorithms and state-space estimation math matrices.
* `tests/`: Automated unit testing framework suites utilizing Google Test (GTest).

## 🚀 Building and Running Locally

### Prerequisites
* CMake (Version 3.22 or higher)
* C++17 compliant compiler (`GCC`, `Clang`, or `MSVC`)

### Compile Steps
```bash
# Configure the build system
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release

# Compile the binaries
cmake --build build

# Execute the simulation engine
./build/bin/run_simulation
```

## 🤖 Continuous Integration (CI)
This codebase utilizes **GitHub Actions** to automate continuous integration. On every push or pull request, cloud pipelines automatically build the engine under both `GCC` and `Clang` toolchains using strict warning flags (`-Wall -Wextra -Werror`) to guarantee safety-critical code compilation stability.
