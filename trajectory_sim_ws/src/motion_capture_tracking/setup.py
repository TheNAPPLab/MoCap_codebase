from setuptools import setup
import os

package_name = 'motion_capture_tracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', 
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='timothy.churchillr@gmail.com',
    description='Trajectory generation and path publishing nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'min_snap_trajectory_publisher = motion_capture_tracking.min_snap_trajectory_publisher:main',
            'trajectory_publisher = motion_capture_tracking.trajectory_publisher:main',
        ],
    },
)
