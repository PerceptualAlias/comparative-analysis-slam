from setuptools import setup
import os
from glob import glob

package_name = 'kitti_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This installs your 00.txt data file
        (os.path.join('share', package_name, 'data'), glob('data/*.txt')),
        # This installs your EKF config file
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yash Khanduja',
    maintainer_email='yash.2428020318@muj.manipal.edu',
    description='KITTI to ROS 2 Converter with EKF support',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'command_name = folder_name.file_name:function_name'
            'kitti_play = kitti_converter.kitti_to_ros2:main',
        ],
    },
)