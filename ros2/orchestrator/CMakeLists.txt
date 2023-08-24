cmake_minimum_required(VERSION 3.10)
project(orchestrator)

find_package(aduulm_cmake_tools)
if (aduulm_cmake_tools_FOUND)
    aduulm_init_ros2_package()
else ()
    find_package(ament_cmake REQUIRED)
endif ()

install(DIRECTORY
        schemas
        config
        DESTINATION share/${PROJECT_NAME}
)

if (aduulm_cmake_tools_FOUND)
    aduulm_install_ros2_python_code("orchestrator")
    aduulm_create_ros2_package()
else ()
    ament_python_install_package("orchestrator")
    ament_package()
endif ()
