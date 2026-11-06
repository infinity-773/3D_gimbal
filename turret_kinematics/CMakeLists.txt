cmake_minimum_required(VERSION 3.16)
project(turret_kinematics LANGUAGES CXX)

add_library(turret_kinematics INTERFACE)
target_include_directories(turret_kinematics INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(turret_kinematics INTERFACE cxx_std_20)

enable_testing()
add_executable(test_turret_kinematics tests/test_turret_kinematics.cpp)
target_link_libraries(test_turret_kinematics PRIVATE turret_kinematics)
add_test(NAME turret_kinematics COMMAND test_turret_kinematics)