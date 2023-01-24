cmake_minimum_required(VERSION 2.8)

project(test_project)
add_executable(${PROJECT_NAME} "main.cpp")

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries( test_project ${OpenCV_LIBS} )

find_package(realsense2)
target_link_libraries(${PROJECT_NAME} PRIVATE realsense2::realsense2)



