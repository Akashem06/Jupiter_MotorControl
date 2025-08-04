cmake_minimum_required(VERSION 3.10)
project(Jupiter VERSION 1.0)

set(CMAKE_C_STANDARD 11)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

file(GLOB_RECURSE CORE_SOURCES 
    "core/*.c"
    "utils/src/*.c"
)

file(GLOB_RECURSE SIM_SOURCES 
    "simulation/src/*.c"
    "hal/src/hal_sim.c"
)

file(GLOB TEST_SOURCES 
    "tests/src/*.c"
)

include(FetchContent)
FetchContent_Declare(
    unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG v2.5.2
)
FetchContent_MakeAvailable(unity)

# Core library
add_library(motor_core STATIC ${CORE_SOURCES})

target_include_directories(
    motor_core PUBLIC
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_SOURCE_DIR}/core/inc
    ${CMAKE_SOURCE_DIR}/core/bldc_6step/inc
    ${CMAKE_SOURCE_DIR}/core/foc_pmsm/inc
    ${CMAKE_SOURCE_DIR}/utils/inc
    ${CMAKE_SOURCE_DIR}/hal/inc
)

# Shared libraries for python interfacing (host side)
add_library(svpwm_transform_python_visualizer SHARED 
    ${CMAKE_SOURCE_DIR}/utils/src/transform_utils.c
    ${CMAKE_SOURCE_DIR}/utils/src/svpwm.c
    ${CMAKE_SOURCE_DIR}/utils/src/math_utils.c
)

target_include_directories(
    svpwm_transform_python_visualizer PUBLIC
    ${CMAKE_SOURCE_DIR}/utils/inc
)

add_library(pll_python_visualizer SHARED 
    ${CMAKE_SOURCE_DIR}/utils/src/pll.c
    ${CMAKE_SOURCE_DIR}/utils/src/math_utils.c
)

target_include_directories(
    pll_python_visualizer PUBLIC
    ${CMAKE_SOURCE_DIR}/utils/inc
)

# Simulation executable
add_executable(sim_bldc ${SIM_SOURCES})

target_include_directories(
    sim_bldc PRIVATE
    ${CMAKE_SOURCE_DIR}/simulation/inc
    ${CMAKE_SOURCE_DIR}/hal/inc
)

target_link_libraries(
    sim_bldc 
    PRIVATE
    motor_core
    m
)

# Tests executable
add_executable(run_tests ${TEST_SOURCES})

target_include_directories(
    run_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/tests/inc
    ${CMAKE_SOURCE_DIR}/core/inc
    ${CMAKE_SOURCE_DIR}/core/bldc_6step/inc
    ${CMAKE_SOURCE_DIR}/core/foc_pmsm/inc
    ${CMAKE_SOURCE_DIR}/utils/inc
    ${CMAKE_SOURCE_DIR}/hal/inc
)

target_link_libraries(
    run_tests
    PRIVATE
    motor_core
    m
    unity
)

# Custom targets for running stuff
add_custom_target(run_simulation
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/simulation/scripts/sim_bridge.py
    DEPENDS sim_bldc
    COMMENT "Running motor simulation..."
)

add_custom_target(run_tests_all
    COMMAND ./run_tests
    DEPENDS run_tests
    COMMENT "Running tests..."
)

add_custom_target(run_viz
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/simulation/scripts/visualization.py
    COMMENT "Running visualization demo..."
)

add_custom_target(doxygen
    COMMAND doxygen ../doxygen/Doxyfile
    COMMENT "Generating documentation..."
)

add_custom_target(format
    COMMAND find .. -type f -name "*.c" -exec clang-format -i -style=file {} +
    COMMAND find .. -type f -name "*.h" -exec clang-format -i -style=file {} +
    COMMENT "Formatting source files..."
)
