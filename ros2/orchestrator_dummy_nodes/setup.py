from glob import glob
import os
from setuptools import setup

package_name = 'orchestrator_dummy_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'configs'), [
            'configs/camera_input_node_config.json',
            'configs/detector_node_config.json',
            'configs/planning_node_config.json',
            'configs/plausibility_node_config.json',
            'configs/service_caller_node_config.json',
            'configs/service_provider_node_config.json',
            'configs/service_test_launch_config.json',
            'configs/time_sync_test_launch_config.json',
            'configs/tracking_node_config.json',
            'configs/tracking_example_launch_config.json'])

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gja38',
    maintainer_email='gja38@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parallel_undeterministic_publisher = orchestrator_dummy_nodes.parallel_undeterministic_publisher:main',
            'multi_subscriber = orchestrator_dummy_nodes.multi_subscriber:main',
            'interceptor = orchestrator_dummy_nodes.interceptor:main',
            'graph_analysis = orchestrator_dummy_nodes.graph_analysis:main',
            'timed_sensor_publisher = orchestrator_dummy_nodes.timed_sensor_publisher:main',
            'simple_timer_publisher = orchestrator_dummy_nodes.simple_timer_publisher:main',
            'tracking_subscriber = orchestrator_dummy_nodes.tracking_subscriber:main',
            'detector = orchestrator_dummy_nodes.detector:main',
            'test = orchestrator_dummy_nodes.test:main',
            'plausibility_node = orchestrator_dummy_nodes.plausibility_node:main',
            'forwarding_node = orchestrator_dummy_nodes.forwarding_node:main',
            'service_caller_node = orchestrator_dummy_nodes.service_caller_node:main',
            'service_provider_node = orchestrator_dummy_nodes.service_provider_node:main',
            'orchestrator = orchestrator_dummy_nodes.orchestrator_node:main',
            'camera_input_node = orchestrator_dummy_nodes.camera_input_node:main'
        ],
    },
)
