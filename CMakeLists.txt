cmake_minimum_required(VERSION 3.24)
project(LidarBaslerMapper)

add_compile_definitions(MAIN_DIR=\"${LidarBaslerMapper_SOURCE_DIR}\")

add_subdirectory(BaslerCalibration)
add_subdirectory(ImagePCDGrabber)
add_subdirectory(Mapper)
