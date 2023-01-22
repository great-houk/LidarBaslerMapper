cmake_minimum_required(VERSION 3.24)
project(ImagePCDGrabber)

set(CMAKE_CXX_STANDARD 17)

add_compile_definitions(LIDAR_DIR=\"${ImagePCDGrabber_SOURCE_DIR}\")

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

link_directories(~/Livox-SDK/build/sdk_core/)

find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()
include_directories(${Pylon_INCLUDE_DIRS})
link_directories(${Pylon_LIBRARY_DIRS})
add_definitions(${Pylon_DEFINITIONS})

add_executable(ImagePCDGrabber main.cpp)

target_link_libraries(ImagePCDGrabber Threads::Threads)
target_link_libraries(ImagePCDGrabber ${PCL_LIBRARIES})
target_link_libraries(ImagePCDGrabber ${Pylon_LIBRARIES})
target_link_libraries(ImagePCDGrabber liblivox_sdk_static.a)