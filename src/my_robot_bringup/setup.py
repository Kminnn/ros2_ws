from setuptools import setup

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)

