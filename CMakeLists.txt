cmake_minimum_required(VERSION 3.25)

project(PCL_test)
set(PROJECT_NAME "PCL_test")

set(CMAKE_CXX_STANDARD 14)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(${PROJECT_NAME} STREQUAL "PCL_test")
    add_executable(PCL_test newCluster.cpp include/dbscan.hpp include/PCL_TEST_HEADER.h include/give_color.hpp include/statistics.hpp include/optics_new.hpp)
elseif(${PROJECT_NAME} STREQUAL "PCL_test2")
    add_executable(PCL_test2 convert.cpp include/PCL_TEST_HEADER.h)
else()
    message("Unknown project name.")
endif()

target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)


