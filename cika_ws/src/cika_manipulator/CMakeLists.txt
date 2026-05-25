cmake_minimum_required(VERSION 3.22)
project(cika_manipulator)

# 1. Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

# 3. Generate interfaces (Define types first)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RobotarmTask.action"
  DEPENDENCIES std_msgs geometry_msgs
)

# 4. BUILD THE EXECUTABLES (Define the targets)
add_executable(task_server src/task_server.cpp)
ament_target_dependencies(task_server 
  rclcpp 
  rclcpp_action 
  rclcpp_components 
  moveit_ros_planning_interface 
  geometry_msgs
)

add_executable(task_client src/task_client.cpp)
ament_target_dependencies(task_client 
  rclcpp 
  rclcpp_action 
)

# 5. LINK THE INTERFACES (Now that task_server and task_client exist)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(task_server "${cpp_typesupport_target}")
target_link_libraries(task_client "${cpp_typesupport_target}")

# 6. Install rules
install(TARGETS
  task_server
  task_client
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE
  )
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# 7. Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 8. ament_package MUST be at the very end
ament_package()