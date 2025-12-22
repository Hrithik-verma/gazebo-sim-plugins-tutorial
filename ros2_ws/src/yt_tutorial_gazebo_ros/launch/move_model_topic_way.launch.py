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
        'move_model_topic_way.sdf',
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


    # gz_to_ros_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='z_velocity_bridge',
    #     output='screen',
    #     arguments=[
    #         '/cmd_vel_z@std_msgs/msg/Float64@gz.msgs.Double['
    #     ],
    # )

    config_file_path = os.path.join(
    get_package_share_directory('yt_tutorial_gazebo_ros'),
    'configs',
    'model_topic_way_bridge.yaml'
    )

    gz_to_ros_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='z_velocity_bridge',
    output='screen',
    parameters=[{
        'config_file': config_file_path
    }],
)

    return LaunchDescription([set_plugin_path, gz_sim, gz_to_ros_bridge])
