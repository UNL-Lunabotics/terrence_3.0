cmake_minimum_required(VERSION 3.8)
project(camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realsense2 QUIET)

if(NOT realsense2_FOUND)
  find_library(REALSENSE2_LIB realsense2)
  if(NOT REALSENSE2_LIB)
    message(FATAL_ERROR "librealsense2 not found")
  endif()
  set(realsense2_FOUND TRUE)
endif()

add_executable(realsense_node src/realsense_node.cpp)

ament_target_dependencies(realsense_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(realsense_node
  ${OpenCV_LIBS}
  ${REALSENSE2_LIB}
)

# Install the executable
install(TARGETS
  realsense_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()