from setuptools import setup
import os
from glob import glob

package_name = 'humanoid_locomotion'

setup(
    name=package_name,
    version='1.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- 关键：确保 launch 和 config 文件夹被安装到系统路径 ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DENG QINGSONG',
    maintainer_email='1243235936@qq.com',
    description='逐际机器人全身运动控制包',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 格式: '启动快捷命令 = 包名.文件名:main函数'
            'facial_driver = humanoid_locomotion.facial_driver:main',
            'facial_control_ui = humanoid_locomotion.facial_control_ui:main',
            'gesture_control_gui_plus = humanoid_locomotion.gesture_control_gui_plus:main'


        ],
    },
)