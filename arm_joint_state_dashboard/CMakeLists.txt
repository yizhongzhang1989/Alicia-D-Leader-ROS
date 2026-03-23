cmake_minimum_required(VERSION 3.8)
project(arm_joint_state_dashboard)

find_package(ament_cmake REQUIRED)

# Install Python node
install(PROGRAMS
  scripts/dashboard_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install static web files
install(DIRECTORY static/
  DESTINATION share/${PROJECT_NAME}/static
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
