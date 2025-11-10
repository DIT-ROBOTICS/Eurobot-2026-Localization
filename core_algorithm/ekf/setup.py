import os  # Import os module
from setuptools import find_packages, setup
from glob import glob

package_name = 'ekf'

launch_files = glob('launch/*.launch') + glob('launch/*.py')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         launch_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='zhen.cc.1207@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_node = ekf.ekf_node:main',
            'test_br = ekf.test_br:main',
            'fake_cam = ekf.fake_cam:main',
        ],
    },
)
