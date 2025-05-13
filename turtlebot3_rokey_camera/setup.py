from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_rokey_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='weed',
    maintainer_email='jjoonmo0212@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_compensation = turtlebot3_rokey_camera.image_compensation:main',
            'image_projection = turtlebot3_rokey_camera.image_projection:main',
            'aruco_pose_publisher = turtlebot3_rokey_camera.aruco_pose_publisher:main',
        ],
    },
)
