cmake_minimum_required(VERSION 3.10)
project(orchestrator_dummy_nodes)

find_package(aduulm_cmake_tools)
aduulm_init_ros2_package()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

aduulm_install_ros2_python_code("orchestrator_dummy_nodes")

aduulm_create_ros2_package()
setup_git_hooks(py_FORMATTERS "autopep8")
