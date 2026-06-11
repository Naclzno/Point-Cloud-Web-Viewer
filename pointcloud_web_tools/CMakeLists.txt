cmake_minimum_required(VERSION 3.8)
project(pointcloud_web_tools)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pointcloud_downsampler
  src/pointcloud_downsampler.cpp
)

target_compile_features(pointcloud_downsampler PUBLIC cxx_std_17)
ament_target_dependencies(pointcloud_downsampler
  rclcpp
  sensor_msgs
)

install(TARGETS
  pointcloud_downsampler
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
