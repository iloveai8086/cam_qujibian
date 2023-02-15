cmake_minimum_required(VERSION 3.19)
project(cam_qujibian)

set(CMAKE_CXX_STANDARD 14)


find_package(OpenCV 3.2 REQUIRED)
include_directories(${Opencv_INCLUDE_DIRS})
link_directories(${OpenCV_LIBS})

add_executable(cam_qujibian main.cpp )
target_link_libraries(cam_qujibian ${OpenCV_LIBS})