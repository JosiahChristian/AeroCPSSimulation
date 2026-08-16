cmake_minimum_required(VERSION 3.22)

project(AeroCPSSimulation VERSION 1.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(sim_engine STATIC src/SimulatorEngine.cpp)
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
endif()
