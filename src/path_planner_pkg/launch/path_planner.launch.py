from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planner_pkg',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen'
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='path_planner_pkg',
                    executable='mock_pose_publisher',
                    name='mock_pose_publisher',
                    output='screen'
                ),
            ]
        ),
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
    ])