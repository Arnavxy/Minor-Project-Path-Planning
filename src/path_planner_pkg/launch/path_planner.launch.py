import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_path_planner = get_package_share_directory('path_planner_pkg')

    world = os.path.join(pkg_path_planner, 'worlds', 'city_world.world')

    # Set the GAZEBO_MODEL_PATH to include the path to the models directory
    model_path = os.path.join(pkg_path_planner, 'models')
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    os.environ['GAZEBO_MODEL_PATH'] = f"{model_path}:{gazebo_model_path}"

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Read the model file (treating SDF as URDF content since it has <robot> tag)
    sdf_file = os.path.join(pkg_path_planner, 'models', 'drone', 'drone.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-file', sdf_file,
                           '-entity', 'drone',
                           '-x', '0', '-y', '0', '-z', '1'],
                output='screen'
            )
        ]
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory('path_planner_pkg'),
        'launch',
        'path_planner.rviz')

    # Launch PX4 and MAVROS
    px4_mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path_planner, 'launch', 'px4_mavros.launch.py')
        )
    )

    return LaunchDescription([
        # gzserver_cmd,
        # gzclient_cmd,
        # node_robot_state_publisher,
        # spawn_entity,
        px4_mavros_launch,
        Node(
            package='path_planner_pkg',
            executable='initial_pose_publisher',
            name='initial_pose_publisher',
            output='screen'
        ),
        Node(
            package='path_planner_pkg',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen'
        ),
        Node(
            package='path_planner_pkg',
            executable='path_follower',
            name='path_follower',
            output='screen'
        ),
        Node(
            package='path_planner_pkg',
            executable='costmap_publisher',
            name='costmap_publisher',
            output='screen'
        ),
        # TimerAction(
        #     period=2.0,
        #     actions=[
        #         Node(
        #             package='path_planner_pkg',
        #             executable='mock_pose_publisher',
        #             name='mock_pose_publisher',
        #             output='screen'
        #         ),
        #     ]
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),
    ])