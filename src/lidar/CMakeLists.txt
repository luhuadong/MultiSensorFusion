cmake_minimum_required(VERSION 2.8)
project(LidarDriverSDK)

find_package( Boost REQUIRED  COMPONENTS thread)
find_package( PCL REQUIRED COMPONENTS common )

# set (CMAKE_CXX_FLAGS "-fPIC --std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )

add_subdirectory(src/driver)

include_directories(
    .
    api
    include
    src/driver/include
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME} SHARED
    src/lidar_driver_sdk.cc
    src/tcp_command_client.c
    src/util.c
)

target_link_libraries(${PROJECT_NAME} 
    LidarDriver
    ${Boost_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    Boost::thread
    pcap
)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    find_package(PCL REQUIRED)
    add_executable(LidarDriverTest
        test/test.cc
    )

    target_link_libraries(LidarDriverTest
        ${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${PCL_IO_LIBRARIES}
        Boost::thread
    )
endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})

