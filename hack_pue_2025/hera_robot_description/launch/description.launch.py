from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore
from launch_ros.descriptions import ParameterValue # type: ignore
from launch.substitutions import Command # type: ignore
import os
from ament_index_python.packages import get_package_share_path # type: ignore

def generate_launch_description():
    
    urdf_path = os.path.join(get_package_share_path('hera_robot_description'),
        'urdf',
        'diff_hera.urdf.xacro'
    )

    rviz_path = os.path.join(get_package_share_path('hera_robot_description'),
        'rviz',
        'config.rviz'
    )
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_path],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])
