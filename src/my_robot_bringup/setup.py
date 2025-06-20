from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khemin',
    maintainer_email='your_email@example.com',
    description='Robot bringup package',
    license='MIT',
    tests_require=['pytest'],  # <-- optional; safe to leave out
    entry_points={
        'console_scripts': [
            'odom_publisher = my_robot_bringup.odom_publisher:main',
            'scan_odom_republisher = my_robot_bringup.scan_odom_to_odom:main',
        ],
    },
)

