from setuptools import find_packages, setup

package_name = 'wall_following'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wall_following_with_slam.launch.py']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rmitaiil',
    maintainer_email='rmitaiil@todo.todo',
    description='Wall-following behavior node using LiDAR scan data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follow_node = wall_following.wall_follow_node:main',
            'mecanum_navigator = wall_following.mecanum_navigator:main',
        ],
    },
)
