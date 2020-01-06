cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(lidar_detection)

find_package(PCL 1.2 REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable(lidar_detection 
    src/main.cpp
)
target_link_libraries (lidar_detection ${PCL_LIBRARIES})