import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'healthcheck'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch/firmware', '*launch.[pxy][yma]*'))),
        # Include all config files.
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yml'))),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'obstacle_detector',
        'visualization_msgs',
        'std_msgs',
        'tf2_ros',
        'custom_interfaces',
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='jossiew621@gapp.nthu.edu.tw',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'healthcheck_node = healthcheck.healthcheck:main'
        ],
    },
)
