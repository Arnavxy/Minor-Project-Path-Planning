from setuptools import find_packages, setup

package_name = 'path_planner_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/path_planner.launch.py', 'launch/px4_mavros.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/drone_world.world', 'worlds/empty_world.world', 'worlds/obstacle_world.world', 'worlds/city_world.world']),
        ('share/' + package_name + '/models/drone', ['models/drone/drone.sdf', 'models/drone/drone.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arnav',
    maintainer_email='arnav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_planner_node = path_planner_pkg.path_planner_node:main',
            'mock_pose_publisher = path_planner_pkg.mock_pose_publisher:main',
            'path_follower = path_planner_pkg.path_follower:main',
            'costmap_publisher = path_planner_pkg.costmap_publisher:main',
            'initial_pose_publisher = path_planner_pkg.initial_pose_publisher:main',
        ],
    },
)
