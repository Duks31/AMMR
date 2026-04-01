cmake_minimum_required(VERSION 3.8)
project(robot_arm_remote)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 1. FIND REQUIRED PACKAGES ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_arm_msgs REQUIRED)

# --- 2. BUILD THE TASK SERVER ---
add_executable(task_server src/task_server.cpp)
ament_target_dependencies(task_server 
  rclcpp 
  rclcpp_action 
  rclcpp_components 
  moveit_ros_planning_interface 
  geometry_msgs
  robot_arm_msgs
)

# --- 3. BUILD THE AUTONOMOUS CLIENT ---
add_executable(task_client src/task_client.cpp)
ament_target_dependencies(task_client 
  rclcpp 
  rclcpp_action 
  robot_arm_msgs
)

# --- 4. INSTALL EXECUTABLES ---
# This tells ROS 2 where to put the compiled programs so you can run them from the terminal
install(TARGETS
  task_server
  task_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()