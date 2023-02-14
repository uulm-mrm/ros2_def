from setuptools import setup

package_name = 'orchestrator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/schemas', ['schemas/launch_config_schema.json',
                                                'schemas/node_config_schema.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonas Otto',
    maintainer_email='edwin.otto@uni-ulm.de',
    description='Library for sequence-deterministic ROS node execution',
    license='WTFPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'remapping_generation = orchestrator.remapping_generation:main',
        ],
    },
)
