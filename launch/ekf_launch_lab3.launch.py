from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    config = os.path.join(get_package_share_directory('labs'), 'launch','ekf_lab3.yaml')
    #config ='/home/student/ros2_ws/src/labs/launch/ekf_lab3.yaml'
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config],
        ),
    ])

