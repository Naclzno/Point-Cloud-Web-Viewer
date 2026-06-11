cmake_minimum_required(VERSION 3.8)
project(pointcloud_web_tools)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pointcloud_downsampler
  src/pointcloud_downsampler.cpp
)

add_executable(pointcloud_ws_server
  src/pointcloud_ws_server.cpp
)

target_compile_features(pointcloud_downsampler PUBLIC cxx_std_17)
ament_target_dependencies(pointcloud_downsampler
  rclcpp
  sensor_msgs
)

target_compile_features(pointcloud_ws_server PUBLIC cxx_std_17)
target_include_directories(pointcloud_ws_server PRIVATE
  ${Boost_INCLUDE_DIRS}
)
ament_target_dependencies(pointcloud_ws_server
  rclcpp
  sensor_msgs
)
target_link_libraries(pointcloud_ws_server
  Boost::system
)

install(TARGETS
  pointcloud_downsampler
  pointcloud_ws_server
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/recent_bag_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
