from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'smrr_navigation'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name],
    package_data={
        package_name: ['models/*.onnx', 'models/*.onnx.data', 'models/*.pt'],
    },
    include_package_data=True,

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch') , glob('launch/*')),
        (os.path.join('share',package_name,'config') , glob('config/*')),
        (os.path.join('share',package_name,'maps') , glob('maps/*')),
        (os.path.join('share',package_name,'srv') , glob('srv/*.srv')),
        (os.path.join('share',package_name,'models') , glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nadil',
    maintainer_email='gunawardaneernh.21@uom.lk',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'startup_localizer = smrr_navigation.startup_localizer:main',
            'named_goal_server = smrr_navigation.named_goal_server:main',
            'named_goal_client = smrr_navigation.named_goal_client:main',
            'location_subscriber = smrr_navigation.location_subscriber:main',
            'smrr_multifloor_bt_navigator = smrr_navigation.smrr_multifloor_bt_navigator:main',
            'door_classifier_node = smrr_navigation.door_classifier_node:main',
            'test_floor_vision = smrr_navigation.test_floor_vision:main',
            'floor_arrival_server = smrr_navigation.floor_arrival_server:main',
        ],
    },
)
