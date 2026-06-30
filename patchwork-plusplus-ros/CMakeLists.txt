cmake_minimum_required(VERSION 3.8)
project(patchworkpp)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(OpenMP)

if(OpenMP_CXX_FOUND)
  add_compile_options(${OpenMP_CXX_FLAGS})
  add_link_options(${OpenMP_EXE_LINKER_FLAGS})
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(PATCHWORKPP_DEPS
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
)

add_executable(demo src/demo.cpp)
ament_target_dependencies(demo ${PATCHWORKPP_DEPS})
target_link_libraries(demo ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} Boost::filesystem Boost::system)

add_executable(offline_kitti src/offline_kitti.cpp)
ament_target_dependencies(offline_kitti ${PATCHWORKPP_DEPS})
target_link_libraries(offline_kitti ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} Boost::filesystem Boost::system)

add_executable(video src/video.cpp)
ament_target_dependencies(video ${PATCHWORKPP_DEPS})
target_link_libraries(video ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} Boost::filesystem Boost::system)

install(TARGETS demo offline_kitti video
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${PATCHWORKPP_DEPS})

ament_package()
