cmake_minimum_required(VERSION 3.1)
project(objectFitting)

set(CMAKE_CXX_STANDARD 11)


find_package(PCL 1.9 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(objectFitting objectFitting.cpp recFitting.cpp recFitting.h)

target_link_libraries(objectFitting ${PCL_LIBRARIES})
