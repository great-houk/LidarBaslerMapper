cmake_minimum_required(VERSION 3.24)
project(BaslerCalibration)

set(CMAKE_CXX_STANDARD 17)

add_compile_definitions(BASLER_DIR=\"${BaslerCalibration_SOURCE_DIR}\")

find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif ()
include_directories(${Pylon_INCLUDE_DIRS})
link_directories(${Pylon_LIBRARY_DIRS})
add_definitions(${Pylon_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

add_executable(BaslerCalibration main.cpp)

target_link_libraries(BaslerCalibration ${Pylon_LIBRARIES})
target_link_libraries(BaslerCalibration ${OpenCV_LIBRARIES})
