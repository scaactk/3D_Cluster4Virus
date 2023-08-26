cmake_minimum_required(VERSION 3.25)
project(PCL_test)

set(CMAKE_CXX_STANDARD 14)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(PCL_test PCL_dbscan.cpp dbscan.hpp PCL_TEST_HEADER.h give_color.hpp dbscan_kdtree.hpp)

target_link_libraries (PCL_test ${PCL_LIBRARIES})
install(TARGETS PCL_test RUNTIME DESTINATION bin)


