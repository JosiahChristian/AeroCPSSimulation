cmake_minimum_required(VERSION 3.22)

# Project Metadata
project(AeroCPSSimulation
    VERSION 1.0.0
    DESCRIPTION "High-fidelity cyber-physical aerospace simulation engine"
    LANGUAGES CXX
)

# Enforce Modern C++ Standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set Output Directories for compiled binaries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Collects all core source files from the src directory
file(GLOB_RECURSE SIM_SOURCES "src/*.cpp")

# Main Simulation Library (Can be empty initially, handles our future classes)
if(SIM_SOURCES)
    add_library(sim_engine SHARED ${SIM_SOURCES})
else()
    # Fallback if src folder is still empty to prevent CMake from crashing
    add_library(sim_engine INTERFACE)
endif()

# Target Include Directories
target_include_directories(sim_engine
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Main Executable Application
add_executable(run_simulation apps/main.cpp)
target_link_libraries(run_simulation PRIVATE sim_engine)