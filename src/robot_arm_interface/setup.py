from setuptools import find_packages, setup

package_name = 'robot_arm_interface'

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
            'arduino_serial_node = robot_arm_interface.arduino_serial_node:main',
            'arm_kinematics_node = robot_arm_interface.arm_kinematics_node:main',
            'arm_trajectory_node = robot_arm_interface.arm_trajectory_node:main',
            'beacon_control_node = robot_arm_interface.beacon_control_node:main',
        ],
    },
)
