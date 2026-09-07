cmake_minimum_required(VERSION 3.16)
project(autonomy_platform LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Simulated CAN / TWAI bus, a small standalone C library.
add_library(can_bus STATIC
    src/can_bus.c
)
target_include_directories(can_bus PUBLIC src)

# Mission computer simulation executable.
add_executable(mission_computer
    src/main.cpp
)
target_link_libraries(mission_computer PRIVATE can_bus)

# Test executables.
enable_testing()

add_executable(test_ekf tests/test_ekf.cpp)
add_executable(test_astar tests/test_astar.cpp)

add_test(NAME test_ekf COMMAND test_ekf)
add_test(NAME test_astar COMMAND test_astar)
