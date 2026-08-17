cmake_minimum_required(VERSION 3.22)

project(AeroCPSSimulation VERSION 1.13.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(AEROCPS_ENABLE_SANITIZERS "Enable address and undefined-behavior sanitizers" OFF)

if(AEROCPS_ENABLE_SANITIZERS)
    if(MSVC)
        message(FATAL_ERROR "AEROCPS_ENABLE_SANITIZERS is supported with GCC and Clang")
    endif()
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
endif()

add_library(sim_engine STATIC
    src/AerodynamicModel.cpp
    src/FixedWingReference.cpp
    src/FlightModels.cpp
    src/NumericalIntegrator.cpp
    src/RigidBodyDynamics.cpp
    src/SimulationScenario.cpp
    src/SimulatorEngine.cpp
)
target_include_directories(sim_engine PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(sim_engine PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(sim_engine PRIVATE /W4 /permissive-)
else()
    target_compile_options(sim_engine PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

add_executable(run_simulation apps/main.cpp)
target_link_libraries(run_simulation PRIVATE sim_engine)

add_executable(benchmark_simulation apps/benchmark.cpp)
target_link_libraries(benchmark_simulation PRIVATE sim_engine)

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

    add_executable(aerodynamic_tests tests/test_aerodynamics.cpp)
    target_link_libraries(aerodynamic_tests PRIVATE sim_engine)
    add_test(NAME aerodynamic_tests COMMAND aerodynamic_tests)

    add_executable(telemetry_contract_tests tests/test_telemetry_contract.cpp)
    target_link_libraries(telemetry_contract_tests PRIVATE sim_engine)
    add_test(NAME telemetry_contract_tests COMMAND telemetry_contract_tests)

    add_executable(fixed_wing_reference_tests tests/test_fixed_wing_reference.cpp)
    target_link_libraries(fixed_wing_reference_tests PRIVATE sim_engine)
    add_test(NAME fixed_wing_reference_tests COMMAND fixed_wing_reference_tests)

    add_test(NAME performance_smoke COMMAND benchmark_simulation)
    set_tests_properties(performance_smoke PROPERTIES TIMEOUT 30)
endif()

install(TARGETS sim_engine run_simulation benchmark_simulation
    EXPORT AeroCPSSimulationTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

configure_package_config_file(
    cmake/AeroCPSSimulationConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/AeroCPSSimulationConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/AeroCPSSimulation
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/AeroCPSSimulationConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/AeroCPSSimulationConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/AeroCPSSimulationConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/AeroCPSSimulation
)
install(EXPORT AeroCPSSimulationTargets
    FILE AeroCPSSimulationTargets.cmake
    NAMESPACE AeroCPS::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/AeroCPSSimulation
)
