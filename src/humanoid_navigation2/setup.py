import os
from glob import glob
from setuptools import setup

package_name = 'humanoid_navigation2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # 基础资源索引
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        
        # 安装 Launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 安装配置文件 (YAML)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # 安装行为树文件 (XML) - 在你的图中位于 config/behavior_tree 下
        (os.path.join('share', package_name, 'behavior_tree'), glob('config/behavior_tree/*.xml')),
        
        # 安装地图文件
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'pcd'), glob('pcd/*')),

        #安装 rviz 目录
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Humanoid Navigation Package with Stair Handling',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 定义可执行命令名 = 包路径.脚本名:入口函数
            'map_converter = humanoid_navigation2.map_converter:main',
            'lidar_imu_time_calibrator = humanoid_navigation2.lidar_imu_time_calibrator:main',
            'measure_timestamp_diff = humanoid_navigation2.measure_timestamp_diff:main',
            'save_pcd_map = humanoid_navigation2.save_pcd_map:main',
            'map_coordinate_viewer = humanoid_navigation2.map_coordinate_viewer:main',
            'periodic_clearing_publisher = humanoid_navigation2.periodic_clearing_publisher:main',
            'periodic_clearing_3d_publisher = humanoid_navigation2.periodic_clearing_3d_publisher:main',
            'nav2_motion_monitor = humanoid_navigation2.nav2_motion_monitor:main',
            'velocity_tester_gui = humanoid_navigation2.velocity_tester_gui:main',
            'robot_realpose_publisher = humanoid_navigation2.robot_realpose_publisher:main',

        ],
    },
)