cmake_minimum_required(VERSION 3.24)
project(LiveView)

add_compile_definitions(VIEW_DIR=\"${LiveView_SOURCE_DIR}\")

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

add_executable(LiveView main.cpp ../Mapper/projectPoints.cpp ../CaptureLidarBuffered/LidarCapture.cpp)

target_link_libraries(LiveView Threads::Threads)
target_link_libraries(LiveView ${OpenCV_LIBRARIES})
target_link_libraries(LiveView ${PCL_LIBRARIES})
target_link_libraries(LiveView ${Pylon_LIBRARIES})
target_link_libraries(LiveView yaml-cpp)
target_link_libraries(LiveView liblivox_sdk_static.a)