cmake_minimum_required(VERSION 3.10)
project(orchestrator_rosbag_player)

find_package(aduulm_cmake_tools)
aduulm_init_ros2_package()

install(PROGRAMS
        orchestrator_rosbag_player/rosbag_player.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME rosbag_player
        )

aduulm_install_ros2_python_code("orchestrator_rosbag_player")

aduulm_create_ros2_package()
