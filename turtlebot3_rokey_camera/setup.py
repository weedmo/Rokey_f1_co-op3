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
            'birds_eye_view = turtlebot3_rokey_camera.birds_eye_view:main',
            'auto_birdseye_from_yaml = turtlebot3_rokey_camera.auto_birdseye_from_yaml:main',
            'aruco_pose_publisher = turtlebot3_rokey_camera.aruco_pose_publisher:main',
            'image_preprocessor = turtlebot3_rokey_camera.image_preprocessor:main',
            'sobel_curve_direction = turtlebot3_rokey_camera.sobel_curve_direction:main',
        ],
    },
)
