from setuptools import setup
import os
from glob import glob

package_name = 'humanoid_websocket'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), 
         glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='WebSocket interface for humanoid robot with ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'websocket_client_node = humanoid_websocket.websocket_client:main',
            'websocket_server_node = humanoid_websocket.websocket_server:main',
            'message_bridge_node = humanoid_websocket.message_bridge:main',
            'data_integration_node = humanoid_websocket.data_integration_node:main',
            'test_data_integration_node = humanoid_websocket.test_data_integration_node:main',
            'data_storage_manager_node = humanoid_websocket.data_storage_manager:main',
            'synthetic_data_publisher = humanoid_websocket.synthetic_data_publisher:main',
            'omni_teleop_keyboard = humanoid_websocket.omni_teleop_keyboard:main',
        ],
    },
)