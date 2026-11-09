cmake_minimum_required(VERSION 3.10)

project(VisionGimbal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio)
find_package(Threads REQUIRED)

add_subdirectory(Serial)
add_subdirectory(turret_kinematics)

add_executable(vision_gimbal
    vision_gimbal.cpp
    Debouncer.cpp
    PIDController.cpp
    MathUtils.cpp)
target_include_directories(vision_gimbal PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Serial
    ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision_gimbal PRIVATE
    ${OpenCV_LIBS} TRoMaC::serial turret_kinematics Threads::Threads)
