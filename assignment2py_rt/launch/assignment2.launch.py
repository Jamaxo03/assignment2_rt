import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    simulation_pkg_path = get_package_share_directory('bme_gazebo_sensors')
    
    
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_pkg_path, 'launch', 'spawn_robot.launch.py')
        )
    )

    ui_node = Node(
        package='assignment2py_rt',
        executable='ui_node',
        name='ui_control',
        output='screen',
        prefix=["xterm -e"], 
        shell=True
    )

    safety_node = Node(
        package='assignment2py_rt',
        executable='safety_node',
        name='safety_monitor',
        output='screen'
    )

    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(simulation_launch)
    launchDescriptionObject.add_action(ui_node)
    launchDescriptionObject.add_action(safety_node)
    
    return launchDescriptionObject