cmake_minimum_required(VERSION 3.2)

project(HIKROBOT_Camera_Driver VERSION 0.0.1 LANGUAGES CXX C)

set(CMAKE_BUILD_TYPE "Debug")

set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

include_directories(
    /opt/MVS/include/
    src
)

link_directories(
    /opt/MVS/lib/64/
    src
)

add_subdirectory(src)
add_subdirectory(demo)