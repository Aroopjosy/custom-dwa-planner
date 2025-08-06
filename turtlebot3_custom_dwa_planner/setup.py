from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot3_custom_dwa_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aroop',
    maintainer_email='aroop.josy@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_node = turtlebot3_custom_dwa_planner.odom_node:main',
            'dwa_planner = turtlebot3_custom_dwa_planner.dwa_planner_node:main',
            'test_node = turtlebot3_custom_dwa_planner.test_dwa_node:main',
            'planner = turtlebot3_custom_dwa_planner.planner:main',

        ],
    },

    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ]
)
