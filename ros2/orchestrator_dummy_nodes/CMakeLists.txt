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
install(PROGRAMS
        orchestrator_dummy_nodes/double_timer_publisher.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME double_timer_publisher
)
install(PROGRAMS
        orchestrator_dummy_nodes/configurable_forwarding_node.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME configurable_forwarding_node
)
install(PROGRAMS
        orchestrator_dummy_nodes/detector.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME detector
)
install(PROGRAMS
        orchestrator_dummy_nodes/tracking_subscriber.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME tracking_subscriber
)
install(PROGRAMS
        orchestrator_dummy_nodes/plausibility_node.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME plausibility_node
)
install(PROGRAMS
        orchestrator_dummy_nodes/simple_timer_publisher.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME simple_timer_publisher
)


aduulm_install_ros2_python_code("orchestrator_dummy_nodes")

aduulm_create_ros2_package()
