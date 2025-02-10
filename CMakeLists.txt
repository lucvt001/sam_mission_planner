cmake_minimum_required(VERSION 3.8)
project(sam_mission_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(smarc_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

set(ros_dependencies
  rclcpp
  std_msgs
  geometry_msgs
  smarc_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(ping_synchronizer src/ping_synchronizer.cpp)
ament_target_dependencies(ping_synchronizer ${ros_dependencies})

add_executable(string_stamped_processing src/string_stamped_processing.cpp)
ament_target_dependencies(string_stamped_processing ${ros_dependencies})

add_executable(leader_motion src/leader_motion.cpp)
ament_target_dependencies(leader_motion ${ros_dependencies})

add_executable(ground_truth_tf2 src/ground_truth_tf2.cpp)
ament_target_dependencies(ground_truth_tf2 ${ros_dependencies})

add_executable(fuse_distance_triangulation src/fuse_distance_triangulation.cpp)
ament_target_dependencies(fuse_distance_triangulation ${ros_dependencies})

add_executable(pid_controller src/pid.cpp src/pid_control_ros.cpp)
ament_target_dependencies(pid_controller ${ros_dependencies})

install(TARGETS
  ping_synchronizer
  string_stamped_processing
  leader_motion
  ground_truth_tf2
  fuse_distance_triangulation
  pid_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include/
  DESTINATION include/
  FILES_MATCHING 
    PATTERN "*.h"
    PATTERN "*.hpp"
)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
