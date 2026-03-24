cmake_minimum_required(VERSION 3.8)
project(alicia_duo_leader_driver)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 添加自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmJointState.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

# 安装Python节点
file(GLOB PYTHON_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py")
foreach(script ${PYTHON_SCRIPTS})
  execute_process(COMMAND chmod +x "${script}")
endforeach()

install(PROGRAMS
  scripts/alicia_driver_node.py
  scripts/arm_read_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装配置文件
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../config/
  DESTINATION share/${PROJECT_NAME}/config
  PATTERN "joint_config.yaml" EXCLUDE
)

ament_package()