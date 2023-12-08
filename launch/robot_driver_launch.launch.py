import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Declare the launch arguments
    ur_type_arg = DeclareLaunchArgument('ur_type', default_value='ur5e')
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.56.101')
    use_fake_hardware_arg = DeclareLaunchArgument('use_fake_hardware', default_value='true')
    initial_joint_controller_arg = DeclareLaunchArgument('initial_joint_controller', default_value='joint_trajectory_controller')

    # Include the ur_robot_driver launch file
    ur_control_launch_file = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'launch',
        'ur_control.launch.py'
    )

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_control_launch_file),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'launch_rviz': 'false',
            'initial_joint_controller': LaunchConfiguration('initial_joint_controller')
        }.items()
    )

    # Include the ur_moveit_config launch file
    ur_moveit_launch_file = os.path.join(
        get_package_share_directory('ur_moveit_config'),
        'launch',
        'ur_moveit.launch.py'
    )

    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_moveit_launch_file),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'initial_joint_controller': LaunchConfiguration('initial_joint_controller'),
            'launch_rviz': 'true',
        }.items()
    )


    # pkg_dir = get_package_share_directory('robot_control')
    # rviz_launch_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', os.path.join(pkg_dir, 'rviz', 'rviz_config_file.rviz')]
    # )
    

    return LaunchDescription([
        ur_type_arg,
        robot_ip_arg,
        use_fake_hardware_arg,
        initial_joint_controller_arg,
        ur_control,
        ur_moveit,
        ])