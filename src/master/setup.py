from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'master'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giacomo',
    maintainer_email='giacomo.antonelli3@gmail.com',
    description='Master node for P-Body Robot',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_driver = master.camera_driver:main',
            'robot_controller = master.robot_controller:main',
        ],
    },
)