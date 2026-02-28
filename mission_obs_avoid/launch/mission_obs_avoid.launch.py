from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("mission_obs_avoid")
    default_params = f"{package_share}/config/mission_obs_avoid.yaml"

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    mission_interceptor = Node(
        package="mission_obs_avoid",
        executable="mission_interceptor_node",
        name="mission_interceptor",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    cmd_arbiter = Node(
        package="mission_obs_avoid",
        executable="cmd_vel_arbiter_node",
        name="cmd_vel_arbiter",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            mission_interceptor,
            cmd_arbiter,
        ]
    )
