from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'smrr_crowdnav'

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
        (os.path.join('lib', package_name), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='achiraubuntu',
    maintainer_email='achirahansindu52@gmail.com',
    description='Human detection and crowd navigation using YOLO11 and RGBD camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    scripts=[
        'scripts/evaluate_fusion.py',
        'scripts/check_evaluation_ready.py',
    ],
    entry_points={
        'console_scripts': [
            'human_detector = smrr_crowdnav.human_detector:main',
            'human_lidar_matcher = smrr_crowdnav.human_lidar_matcher:main',
            'human_tracker = smrr_crowdnav.human_tracker:main',
            'human_fusion_node = smrr_crowdnav.human_fusion_node:main',
            'human_fusion_kf = smrr_crowdnav.human_fusion_kf_node:main',
        ],
    },
)
