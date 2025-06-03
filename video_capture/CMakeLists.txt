cmake_minimum_required(VERSION 3.8)
project(video_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(sender src/gstreamer_sender.cpp)
ament_target_dependencies(sender rclcpp)

add_executable(receiver src/gstreamer_receiver.cpp)
ament_target_dependencies(receiver rclcpp OpenCV)

install(TARGETS
  sender
  receiver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
