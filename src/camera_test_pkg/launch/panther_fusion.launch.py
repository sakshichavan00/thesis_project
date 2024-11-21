import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument



current_package = 'camera_test_pkg'  # Replace with your package name



def generate_launch_description():
    # paramter file name argument, without the extionsion
    param_file = LaunchConfiguration("param_file")
    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value='panther_fusion_cam',
        description='Name of the parameter file to use, also the name used for the ekf node being run',
    )

    # use os to get the path to the parameter file
    # Use PathJoinSubstitution with PythonExpression to dynamically add the .yaml extension
    param_file_path = PathJoinSubstitution([
        get_package_share_directory(current_package),
        'config',
        PythonExpression(["'",param_file,"'+'.yaml'"])
    ])

    ekf_fusion_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name= param_file,
        parameters=[param_file_path],
    )


    return LaunchDescription([
        param_file_arg,
        ekf_fusion_node  
    ])

