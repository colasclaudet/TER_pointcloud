cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(test_ransac)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (test_ransac test_ransac.cpp)
target_link_libraries (test_ransac ${PCL_LIBRARIES})
