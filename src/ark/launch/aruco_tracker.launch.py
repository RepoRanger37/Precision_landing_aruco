from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory  # Make sure this import is present

def generate_launch_description():
    # Move variable assignment OUTSIDE the return
    package_path = get_package_share_directory('ark')

    return LaunchDescription([
        # Aruco tracker node
        Node(
            package='ark',
            executable='aruco',
            name='aruco',
            output='screen',  
        ),
        Node(
            package='ark',
            executable='cam',
            name='cam',
            output='screen',   
        ),
        Node(
            package='ark',
            executable='fb',
            name='Buffer',
            output='screen',   
        ),
        Node(
            package='ark',
            executable='video',
            name='Recorder',
            output='screen',   
        ),   
    ])

