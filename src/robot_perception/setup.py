from setuptools import find_packages, setup

package_name = 'robot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='imdluser1',
    maintainer_email='go46doq@tum.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_detector_node = robot_perception.object_detector_node:main',
            'color_analyzer_node = robot_perception.color_analyzer_node:main',
            'sensor_fusion_node = robot_perception.sensor_fusion_node:main',
        ],
    },
)
