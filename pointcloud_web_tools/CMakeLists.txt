cmake_minimum_required(VERSION 3.8)
project(pointcloud_web_tools)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/pointcloud_downsampler
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
