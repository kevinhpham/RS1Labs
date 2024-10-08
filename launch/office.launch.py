from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Get the directory of the current package
    pkg_share = get_package_share_directory('labs')
    turtle_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-1.3')
    y_pose = LaunchConfiguration('y_pose', default='-1.3')
    z_pose = LaunchConfiguration('z_pose', default='-0.8')

    # Set the path to the world file within the package
    world_file_path = os.path.join(pkg_share, 'worlds', 'office_small.world')
    
     # Debug info
    print(f"Launching Gazebo with world: {world_file_path}")

    # Optionally, set the Gazebo model path to include the models in your package
    #os.environ['GAZEBO_MODEL_PATH'] = os.path.join(pkg_share, 'models') + ':' + 	os.environ.get('GAZEBO_MODEL_PATH', '')

    # Set the Gazebo launch file from the gazebo_ros package
    gazebo_launch_file_dir = os.path.join(pkg_gazebo_ros, 'launch')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtle_launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
        
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtle_launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose
        }.items()
    )

    # Launch the Gazebo simulation with the specified world
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)



    return ld

