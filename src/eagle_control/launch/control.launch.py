from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    simDir = get_package_share_directory("eagle_sim")
    controlDir = get_package_share_directory("eagle_control")

    simLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simDir, "launch", "dev.launch.py")
        ),
        launch_arguments={"rviz": "false"}.items(),
    )

    centerlineNode = Node(
        package="eagle_control",
        executable="centerlineNode",
        name="centerlineNode",
        output="screen",
    )

    controlNode = Node(
        package="eagle_control",
        executable="controlNode",
        name="controlNode",
        output="screen",
    )

    rvizNode = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(controlDir, "rviz", "base.rviz"),
        ],
        output="screen",
    )

    return LaunchDescription([
        simLaunch,
        centerlineNode,
        controlNode,
        rvizNode
    ])