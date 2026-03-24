cmake_minimum_required(VERSION 3.8)
project(alicia_duo_leader_dashboard)

find_package(ament_cmake REQUIRED)

file(GLOB PYTHON_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py")
foreach(script ${PYTHON_SCRIPTS})
  execute_process(COMMAND chmod +x "${script}")
endforeach()

install(PROGRAMS
  scripts/dashboard_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY static/
  DESTINATION share/${PROJECT_NAME}/static
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
