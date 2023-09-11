cmake_minimum_required(VERSION 3.10)
project(orchestrator)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
        schemas
        config
        DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package("orchestrator")
ament_package()
