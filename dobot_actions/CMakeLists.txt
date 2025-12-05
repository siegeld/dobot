cmake_minimum_required(VERSION 3.8)
project(dobot_actions)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveJoints.action"
)

# Install Python executable
install(PROGRAMS
  scripts/move_joints_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
