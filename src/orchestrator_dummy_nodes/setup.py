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
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))

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
            'serializing_interceptor = orchestrator_dummy_nodes.serializing_interceptor:main',
            'timed_sensor_publisher = orchestrator_dummy_nodes.timed_sensor_publisher:main',
            'tracking_subscriber = orchestrator_dummy_nodes.tracking_subscriber:main',
            'detector = orchestrator_dummy_nodes.detector:main',
            'test = orchestrator_dummy_nodes.test:main',
            'orchestrator = orchestrator_dummy_nodes.orchestrator:main',
            'plausibility_node = orchestrator_dummy_nodes.plausibility_node:main',
        ],
    },
)
