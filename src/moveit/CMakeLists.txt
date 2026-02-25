cmake_minimum_required(VERSION 3.22)
project(moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(moveit_common REQUIRED)
moveit_package() # Configures MoveIt settings
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


# Define the executable
add_executable(node_kinematic_controller src/node_kinematic_controller.cpp)

# Link dependencies
ament_target_dependencies(node_kinematic_controller
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

# Install the executable so "ros2 run" can find it
install(TARGETS
  node_kinematic_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
