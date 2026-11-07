cmake_minimum_required(VERSION 3.16)

project(VisionGimbal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio calib3d)
find_package(Threads REQUIRED)

find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(eigen)
endif()

add_subdirectory(Serial)
add_subdirectory(turret_kinematics)

add_library(energy_gimbal
    target_model.cpp
    target_fitter.cpp
    vision_bridge.cpp
    control_loop.cpp)
target_include_directories(energy_gimbal PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(energy_gimbal PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(energy_gimbal PUBLIC turret_kinematics ${OpenCV_LIBS})

add_executable(test_energy_gimbal tests/test_energy_gimbal.cpp)
target_link_libraries(test_energy_gimbal PRIVATE energy_gimbal)
enable_testing()
add_test(NAME energy_gimbal COMMAND test_energy_gimbal)

add_executable(vision_gimbal vision_gimbal.cpp RingDetector.cpp)
target_include_directories(vision_gimbal PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Serial
    ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision_gimbal PRIVATE
    energy_gimbal TRoMaC::serial Threads::Threads)
