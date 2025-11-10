import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'lidar_localization_pkg'

launch_files = (glob('launch/*.launch') +
                glob('launch/*.py') +
                glob('launch/*.xml'))

firmware_launch_files = (glob('launch/firmware/*.launch') +
                         glob('launch/firmware/*.py') +
                         glob('launch/firmware/*.xml'))

config_files = glob('config/*.yml') + glob('config/*.yaml')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name),
         ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
         launch_files),
        (os.path.join('share', package_name, 'launch', 'firmware'),
         firmware_launch_files),

        (os.path.join('share', package_name, 'config'),
         config_files),
    ],

    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'obstacle_detector',
        'visualization_msgs',
        'std_msgs',
        'tf2_ros',
    ],
    zip_safe=True,
    maintainer='jossiew621',
    maintainer_email='jossiew621@gapp.nthu.edu.tw',
    keywords=['scan', 'beacon', 'localization'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache-2.0 License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Eurobot localization using LiDAR scan, probability and triangulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_localization = lidar_localization_pkg.lidar_member_function:main',
            'circle_publisher = lidar_localization_pkg.probability_circle_publisher:main',
            'pred_publisher   = lidar_localization_pkg.pred_publisher:main',
            'lidar_calibrator = lidar_localization_pkg.lidar_calibrator:main',
        ],
    },
)
