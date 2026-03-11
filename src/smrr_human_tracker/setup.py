from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'smrr_human_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pth')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
        (os.path.join('share', package_name, 'models'), glob('models/*.onnx')),
        (os.path.join('share', package_name, 'models'), glob('models/*.engine')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='achiraubuntu',
    maintainer_email='achirahansindu52@gmail.com',
    description='Human detection and crowd navigation using YOLO11 and RGBD camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_tracker = smrr_human_tracker.human_tracker:main',
            'human_fusion_kf = smrr_human_tracker.human_fusion_kf_node:main',
            'lidar_human_detection = smrr_human_tracker.lidar_human_detection:main',
            'evaluate_kf_only = smrr_human_tracker.scripts.evaluate_kf_only:main',
        ],
    },
)
