from setuptools import setup

package_name = 'orchestrator_rosbag_player'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gja38',
    maintainer_email='edwin.otto@uni-ulm.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosbag_player = orchestrator_rosbag_player.rosbag_player:main'
        ],
    },
)
