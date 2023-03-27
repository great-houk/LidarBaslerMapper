cmake_minimum_required(VERSION 3.24)
project(LidarBuffered)

set(CMAKE_CXX_STANDARD 17)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

link_directories(~/Livox-SDK/build/sdk_core/)

find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
    include("../FindPylon.cmake")
endif ()
include_directories(${Pylon_INCLUDE_DIRS})
link_directories(${Pylon_LIBRARY_DIRS})
add_definitions(${Pylon_DEFINITIONS})

find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})
link_directories(${yaml-cpp_LIBRARY_DIRS})
add_definitions(${yaml-cpp_DEFINITIONS})

find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(pybind11)
pybind11_add_module(pymapper SHARED LidarCapture.cpp py-mapper.cpp)
set_target_properties(pymapper PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/pymapper/")

target_link_libraries(pymapper PRIVATE Threads::Threads)
target_link_libraries(pymapper PRIVATE ${OpenCV_LIBRARIES})
target_link_libraries(pymapper PRIVATE ${PCL_LIBRARIES})
target_link_libraries(pymapper PRIVATE ${Pylon_LIBRARIES})
target_link_libraries(pymapper PRIVATE yaml-cpp)
set_target_properties(pymapper PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
target_link_libraries(pymapper PRIVATE liblivox_sdk_static.a)

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_compile_definitions(pymapper
                           PRIVATE VERSION_INFO=0.1)
