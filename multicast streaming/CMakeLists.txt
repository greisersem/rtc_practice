cmake_minimum_required(VERSION 3.10)
project(multicast_streaming)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

add_executable(streaming multicast_streamer.cpp)
add_executable(ip_streaming ip_cam_to_multicast.cpp)

target_link_libraries(streaming ${OpenCV_LIBS})
target_link_libraries(ip_streaming ${OpenCV_LIBS})