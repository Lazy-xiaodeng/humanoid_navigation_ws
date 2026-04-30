from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'humanoid_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')+ glob('config/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.json')+glob('maps/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS User',
    maintainer_email='ros@example.com',
    description='Humanoid robot navigation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamic_waypoints_manager = humanoid_navigation.dynamic_waypoints_manager:main',
            'navigation_state_manager = humanoid_navigation.navigation_state_manager:main',
        ],
    },
)