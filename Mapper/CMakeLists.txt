cmake_minimum_required(VERSION 3.24)
project(Mapper)

add_compile_definitions(MAPPER_DIR=\"${Mapper_SOURCE_DIR}\")

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

link_directories(~/Livox-SDK/build/sdk_core/)

find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})
link_directories(${yaml-cpp_LIBRARY_DIRS})
add_definitions(${yaml-cpp_DEFINITIONS})

find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(Mapper main.cpp projectPoints.cpp)

target_link_libraries(Mapper ${OpenCV_LIBRARIES})
target_link_libraries(Mapper ${PCL_LIBRARIES})
target_link_libraries(Mapper yaml-cpp)
target_link_libraries(Mapper liblivox_sdk_static.a)