import os
from glob import glob
from setuptools import setup

package_name = 'humanoid_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 拷贝 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 如果 launch 文件夹下有 components 这种子文件夹，也需要显式拷贝
        (os.path.join('share', package_name, 'launch', 'components'), glob('launch/components/*.launch.py')),
        
        # 拷贝 config 文件夹里的所有配置文件
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='Bringup package for humanoid robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 如果 bringup 里有想要直接运行的 python 节点（一般没有），写在这里
        ],
    },
)