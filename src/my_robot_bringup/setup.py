from setuptools import find_packages, setup

package_name = 'my_robot_bringup'

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
    maintainer='khemin',
    maintainer_email='khemin2549@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_odom_publisher = my_robot_bringup.fake_odom_publisher:main',
            'cmd_vel_listener = my_robot_bringup.cmd_vel_listener:main',
            'odom_publisher = my_robot_bringup.odom_publisher:main',
        ],
    },	
)

