import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='imdluser1',
    maintainer_email='go46doq@tum.de',
    description='HSV color detection and 3D block localization for IMDL',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'color_detector_node = robot_perception.color_detector_node:main',
            'hsv_tuner_gui = robot_perception.hsv_tuner_gui:main',
        ],
    },
)