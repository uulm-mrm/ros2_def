cmake_minimum_required(VERSION 3.10)
project(orchestrator)

find_package(aduulm_cmake_tools)
aduulm_init_ros2_package()

install(DIRECTORY
  schemas
  configs
  DESTINATION share/${PROJECT_NAME}
)

aduulm_install_ros2_python_code("orchestrator")

aduulm_create_ros2_package()
