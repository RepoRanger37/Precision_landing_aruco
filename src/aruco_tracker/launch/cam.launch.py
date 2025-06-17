from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory  # Make sure this import is present

def generate_launch_description():
    # Move variable assignment OUTSIDE the return
    package_path = get_package_share_directory('aruco_tracker')

    return LaunchDescription([
        # Aruco tracker node
        Node(
            package='aruco_tracker',
            executable='fb',
            name='fb',
            output='screen',  
        ),
        Node(
            package='aruco_tracker',
            executable='cam',
            name='cam',
            output='screen',   
        ),
        Node(
            package='aruco_tracker',
            executable='aroco_tracker',
            name='aruco',
            output='screen',   
        ),
    ])

