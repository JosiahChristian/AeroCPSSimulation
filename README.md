# AeroCPSSimulation: 6-DOF Closed-Loop Flight Dynamics Engine

![Build Status](https://github.com)
[![Release](https://shields.io)](https://github.com)

A high-fidelity computational aerospace simulation framework modeling vertical flight control laws and dynamic planetary environmental translations in C++. The engine utilizes discrete time-slice integrations to verify autonomous vehicle tracking parameters under variable physics conditions.

Targeted for control systems engineering alignment with the **Old Dominion University (ODU) PhD in Modeling and Simulation Engineering** pipeline.

## 🛸 Core Architectural Capabilities

*   **Closed-Loop Feedback System:** Implements independent proportional control loop matrices to actively minimize systemic altitude error vectors.
*   **Variable Planetary Physics:** Configured to process multi-variable gravity matrices—including structural parameters modeling Earth (-9.81 m/s²) and Mars (-3.711 m/s²) landing conditions.
*   **Automated Continuous Integration:** Linked natively to an enterprise DevOps testing pipeline that automatically verifies cross-compiler compliance across GCC and Clang environments with zero caching overhead.

## 🛠️ Local Compilation Blueprint

The project infrastructure uses the modern CMake build ecosystem to manage code compilation across distinct hardware platforms.

### Prerequisites
* CMake 3.22+
* C++17 Compliant Compiler (GCC / MinGW / Clang)

### Execution Workspace Commands
Run this configuration macro loop inside your local PowerShell or terminal console window to compile and launch your binaries:

```bash
# 1. Generate local build system configuration files
cmake -B build -S . -G "MinGW Makefiles" -DCMAKE_CXX_COMPILER=g++ -DCMAKE_MAKE_PROGRAM=make

# 2. Compile the executable binary file
cmake --build build

# 3. Launch the flight simulation engine
.\build\bin\run_simulation.exe
```

## 📂 Structural Tree Layout

```text
AeroCPSSimulation/
├── .github/workflows/  --> Cloud automation configuration scripts
├── include/            --> Class blueprint declarations (.hpp)
├── src/                --> Algorithmic math implementations (.cpp)
├── apps/               --> Execution runtime trigger (main.cpp)
└── CMakeLists.txt      --> Master compilation rules file
```
