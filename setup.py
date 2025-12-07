from setuptools import find_packages, setup
import os

package_name = 'camera_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ignacy',
    maintainer_email='ignacy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [ 
            'camera_node = camera_subscriber.camera_node:main',
            'point_follower = camera_subscriber.point_follower:main',
        ],
    },
    data_files=[
        # wymagane pliki ROS2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # pliki launch
        (os.path.join('share', package_name, 'launch'), ['launch/turtlebot.launch.py']),
    ],
)

