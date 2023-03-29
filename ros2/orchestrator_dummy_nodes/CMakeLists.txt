cmake_minimum_required(VERSION 3.10)
project(orchestrator_dummy_nodes)

find_package(aduulm_cmake_tools)
aduulm_init_ros2_package()

install(DIRECTORY
  launch
  configs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  orchestrator_dummy_nodes/service_caller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME service_caller_node
)
install(PROGRAMS
  orchestrator_dummy_nodes/service_provider_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME service_provider_node
)
install(PROGRAMS
  orchestrator_dummy_nodes/orchestrator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME orchestrator
)

aduulm_install_ros2_python_code("orchestrator_dummy_nodes")

aduulm_create_ros2_package()
