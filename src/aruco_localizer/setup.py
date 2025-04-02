from setuptools import setup

package_name = 'aruco_localizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/full_localization.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/localization_config.rviz']),
        ('share/' + package_name + '/resource', ['resource/aruco_localizer'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ArUco-based localization using ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stream_to_ros = aruco_localizer.stream_to_ros:main',
            'localize = aruco_localizer.localization_node:main',
            'mavsdk_bridge = aruco_localizer.mavsdk_bridge:main'
        ],
    },
)