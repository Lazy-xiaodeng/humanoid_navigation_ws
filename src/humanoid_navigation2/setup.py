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
    description='Humanoid navigation launch, map and mapping utility package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 建图收尾和地图格式转换工具。
            'save_pcd_map = humanoid_navigation2.save_pcd_map:main',
            'pcd_converter = humanoid_navigation2.pcd_converter:main',
        ],
    },
)
