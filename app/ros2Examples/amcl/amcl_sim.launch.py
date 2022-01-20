import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # nav2_yaml = os.path.join(os.getcwd(), 'amcl_params.yaml')
    ld = LaunchDescription([
    Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        prefix=['gdb -ex run --args'],
        on_exit=launch.actions.Shutdown(),
        parameters=[
            {
                'use_sim_time': True
            },
            {
                'set_initial_pose': True
            },
            {
                "initial_pose.x": 0.0
            },
            {
                "initial_pose.y": 0.0
            },
            {
                "initial_pose.z": 0.0
            },
            {
                "initial_pose.yaw": 0.0
            },
            {
                'base_frame_id': "mobile_base_body_link"
            },
            {
                'odom_frame_id': "odom"
            },
            {
                'min_particles': 500
            },
            {
                'max_particles': 2500
            },
            {
                'pf_err': 0.05
            },
            {
                'pf_z': 0.95
            },
            {
                'map_topic': "map"
            },
            {
                'robot_model_type': "differential"
            },
            {
                'alpha1': 2.0
            },
            {
                'alpha2': 2.0
            },
            {
                'alpha3': 0.2
            },
            {
                'alpha4': 0.2
            },
            {
                'max_beams': 50
            },
            {
                'laser_max_range': -1.0
            },
            {
                'laser_min_range': -1.0
            },
            {
                'z_hit': 0.95
            },
            {
                'z_short': 0.1
            },
            {
                'tf_broadcast': True
            },
            {
                'z_max': 0.05
            },
            {
                'z_rand': 0.05
            },
            {
                'sigma_hit': 0.2
            },
            {
                'lambda_short': 0.1
            },
            {
                'laser_model_type': "likelihood_field"
            },
            {
                'laser_likelihood_max_dist': 2.0
            },
            {
                'update_min_d': 0.1
            },
            {
                'update_min_a': 0.1
            },
            {
                'resample_interval': 1
            },
            {
                'transform_tolerance': 0.2
            },
            {
                'recovery_alpha_slow': 0.001
            },
            {
                'recovery_alpha_fast': 0.1
            },
            {
                'scan_topic' :"double_lidar"
            },
            {
                'global_frame_id': "map"
            }
        ]
    ),
    Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['amcl']}]
    )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
