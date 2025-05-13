from setuptools import find_packages, setup

package_name = 'avoidance_roi_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/avoidance_roi_filter/launch', ['launch/avoidance_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='kim@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'roi_filter = avoidance_roi_filter.roi_filter:main',
             'obstacle_stopper = avoidance_roi_filter.obstacle_stopper:main',
             'cmd_vel_publisher = avoidance_roi_filter.cmd_vel_publisher:main',
             'plot_cmd_vel = avoidance_roi_filter.plot_cmd_vel:main',
        ],
    },
)
