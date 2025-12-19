from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    plugin_pkg_path = FindPackageShare('tutorial_gazebo_plugins')

    set_plugin_path = SetEnvironmentVariable(
    'GZ_SIM_SYSTEM_PLUGIN_PATH',
    PathJoinSubstitution([plugin_pkg_path, 'plugins'])
    )

    world_path = os.path.join(
        get_package_share_directory('yt_tutorial_gazebo_ros'),
        'worlds',
        'light_control_direct_ros.sdf',
    )

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # see next section for -v
            'gz_args': f'-r -v4 {world_path}',
        }.items(),
    )




    return LaunchDescription([set_plugin_path, gz_sim])
