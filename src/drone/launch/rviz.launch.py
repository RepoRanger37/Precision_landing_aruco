from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Replace 'robot_viz_cpp' with your actual package name
    pkg_share = get_package_share_directory('drone')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')  # or 'my_robot.urdf' depending on your file name
    rviz_path = os.path.join(pkg_share, 'rviz', 'robot.rviz')  # or your actual rviz config file


    with open(urdf_path, 'r') as urdf_file:
        robot_desc = urdf_file.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')],
        #     output='screen'
        # ),
        # Node(
        #     package='robot_localization',
        #     executable='navsat_transform_node',
        #     name='navsat_transform_node',
        #     parameters=[os.path.join(pkg_share, 'config', 'navsat.yaml')],
        #     output='screen'
        # ),
        Node(
            package='drone',
            executable='gps',
            name='gps_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            output='screen'
        )
    ])

