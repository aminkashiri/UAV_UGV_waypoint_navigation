import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'uav_ugv_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amin',
    maintainer_email='amin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'optitrack_feed_node = uav_ugv_nav.uav.optitrack_feed_node:main',
            'uav_node = uav_ugv_nav.uav.uav_node:main',
            'ugv_node = uav_ugv_nav.ugv.ugv_node:main',
        ],
    },
)
