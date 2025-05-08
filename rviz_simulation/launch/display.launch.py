from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare("voiture_simulation"),
        "urdf",
        "voiture.xacro"
    ])


    pkg_share = get_package_share_directory('voiture_simulation')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'bolide_viewer.rviz')

    return LaunchDescription([
        Node(
            package="joint_state_publisher",  # You can also use "joint_state_publisher"
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen"
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command(["xacro", " ", xacro_file])
            }]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=['-d', rviz_config_path],
            output="screen"

        )
    ])
