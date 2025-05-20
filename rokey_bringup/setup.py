from setuptools import find_packages, setup

package_name = 'rokey_bringup'

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
    maintainer='weed',
    maintainer_email='jjoonmo0212@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_toggle_node = rokey_bringup.obstacle_toggle_node:main',
            'trajectory_client = rokey_bringup.trajectory_client:main',
        ],
    },
)
