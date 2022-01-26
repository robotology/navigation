import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nav2_yaml = os.path.join(os.getcwd(), 'amcl_sim_params.yaml')
    par2 = [{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['amcl']}]
    ld = LaunchDescription([
    Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        on_exit=launch.actions.Shutdown(),
        parameters=nav2_yaml
    ),
    Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters= par2
    )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
