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

include(FetchContent)
FetchContent_Declare(
		yaml-cpp
		GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
		GIT_SHALLOW	ON
		GIT_TAG yaml-cpp-0.6.3
	)
FetchContent_GetProperties(yaml-cpp)
if(NOT yaml-cpp_POPULATED)
	message(STATUS "Populating yaml-cpp...")
	FetchContent_Populate(yaml-cpp)
	# Add here options for yaml-cpp building
	set(YAML_CPP_BUILD_TESTS OFF)
	add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
	message(STATUS "Done.")
endif()
include_directories(BEFORE SYSTEM ${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR}/include)

find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(LiveView main.cpp ../CaptureLidarBuffered/LidarCapture.cpp)

target_link_libraries(LiveView Threads::Threads)
target_link_libraries(LiveView ${OpenCV_LIBRARIES})
target_link_libraries(LiveView ${PCL_LIBRARIES})
target_link_libraries(LiveView ${Pylon_LIBRARIES})
target_link_libraries(LiveView yaml-cpp)
target_link_libraries(LiveView liblivox_sdk_static.a)