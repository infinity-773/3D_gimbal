cmake_minimum_required(VERSION 3.16)

project(CameraCalibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio calib3d)

add_executable(camera_calibration camera_calibration.cpp)
target_include_directories(camera_calibration PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_calibration PRIVATE ${OpenCV_LIBS})
