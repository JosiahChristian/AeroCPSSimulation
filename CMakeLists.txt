cmake_minimum_required(VERSION 3.22)

project(AeroCPSSimulation VERSION 1.5.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(sim_engine STATIC
    src/FlightModels.cpp
    src/NumericalIntegrator.cpp
    src/RigidBodyDynamics.cpp
    src/SimulationScenario.cpp
    src/SimulatorEngine.cpp
)
target_include_directories(sim_engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(MSVC)
    target_compile_options(sim_engine PRIVATE /W4 /permissive-)
else()
    target_compile_options(sim_engine PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

add_executable(run_simulation apps/main.cpp)
target_link_libraries(run_simulation PRIVATE sim_engine)

include(CTest)
if(BUILD_TESTING)
    add_executable(sim_engine_tests tests/test_controller.cpp)
    target_link_libraries(sim_engine_tests PRIVATE sim_engine)
    add_test(NAME sim_engine_tests COMMAND sim_engine_tests)

    add_executable(scenario_tests tests/test_scenario.cpp)
    target_link_libraries(scenario_tests PRIVATE sim_engine)
    add_test(NAME scenario_tests COMMAND scenario_tests)

    add_executable(integrator_tests tests/test_integrator.cpp)
    target_link_libraries(integrator_tests PRIVATE sim_engine)
    add_test(NAME integrator_tests COMMAND integrator_tests)

    add_executable(rigid_body_tests tests/test_rigid_body.cpp)
    target_link_libraries(rigid_body_tests PRIVATE sim_engine)
    add_test(NAME rigid_body_tests COMMAND rigid_body_tests)
endif()
