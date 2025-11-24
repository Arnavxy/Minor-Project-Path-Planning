import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the path to the MAVROS launch file
    mavros_launch_dir = os.path.join(get_package_share_directory('mavros'), 'launch')

    # Get the path to the PX4 launch file
    px4_launch_dir = os.path.join(get_package_share_directory('px4_ros_com'), 'launch')

    # Set the FCU URL for SITL
    fcu_url = LaunchConfiguration('fcu_url', default='udp://:14540@127.0.0.1:14550')

    return LaunchDescription([
        # Launch MAVROS
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(mavros_launch_dir, 'px4.launch')),
            launch_arguments={'fcu_url': fcu_url}.items()
        ),
        # Launch PX4 SITL (Commented out to allow manual launch)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(px4_launch_dir, 'px4_sitl.launch.py')),
        # ),
    ])