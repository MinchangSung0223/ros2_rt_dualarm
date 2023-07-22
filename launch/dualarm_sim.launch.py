import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'dual_arm.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('ros2_rt_dualarm'),
        urdf_file_name)
    rviz_config_file = os.path.join(
        get_package_share_directory('ros2_rt_dualarm'), 'indy7.rviz')        
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='ros2_rt_dualarm',
            executable='ros2_rt_dualarm_ros2_sim',
            name='ros2_rt_dualarm_ros2_sim',
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=['-d', rviz_config_file])            
    ])
