cmake_minimum_required(VERSION 3.29 FATAL_ERROR)

project(PCL_test)
set(PROJECT_NAME "PCL_test")
file(GLOB_RECURSE INCLUDE
        "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
)
file(GLOB_RECURSE SOURCE
        "${CMAKE_CURRENT_SOURCE_DIR}/newCluster.cpp"
)
file(GLOB_RECURSE SOURCE2
        "${CMAKE_CURRENT_SOURCE_DIR}/convert.cpp"
)

set(CMAKE_CXX_STANDARD 17)
message("aaa")
# 首先，确保 CMake 能找到 vcpkg 安装的包
if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
            CACHE STRING "")
endif()
find_package(PCL REQUIRED)
#find_package(PCL REQUIRED COMPONENTS common io) //COMPONENTS: 这个关键字告诉 CMake 你想要加载 PCL 库的特定组件，而不是整个库
find_package(FLANN CONFIG REQUIRED)
message("bbb")
#message(STATUS "PCL DIR ${PCL_DIR}")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DPCL_NO_PRECOMPILE) # seems totally solve the problem of ERROR LNK2019 当定义这个宏时，PCL会在编译时包含模板实现文件（.hpp文件）
#message(STATUS "PCL Include Directories: ${PCL_INCLUDE_DIRS}")
#message(STATUS "PCL Link Directories: ${PCL_LIBRARY_DIRS}")
message(STATUS "PCL Definitions: ${PCL_DEFINITIONS}")
#message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")


if(${PROJECT_NAME} STREQUAL "PCL_test")
    add_executable(${PROJECT_NAME} ${SOURCE} ${INCLUDE})
elseif(${PROJECT_NAME} STREQUAL "PCL_test2")
    add_executable(${PROJECT_NAME} ${SOURCE2} ${INCLUDE})
else()
    message("Unknown project name.")
endif()

target_link_libraries (${PROJECT_NAME} PRIVATE ${PCL_LIBRARIES})
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)


