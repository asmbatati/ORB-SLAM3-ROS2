#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

#---------------------------------------------

    #Essential_paths
    orb_wrapper_pkg = get_package_share_directory('orb_slam3_ros2_wrapper')
#---------------------------------------------

    # LAUNCH ARGS
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    robot_namespace =  LaunchConfiguration('robot_namespace')
    robot_namespace_arg = DeclareLaunchArgument('robot_namespace', default_value="target",
        description='The namespace of the robot')
#---------------------------------------------

    def all_nodes_launch(context, robot_namespace):
        params_file = LaunchConfiguration('params_file')
        
        # Update paths to use the correct location in shared volume
        home_dir = os.environ.get('HOME', '/home/user')
        shared_volume_path = os.path.join(home_dir, 'shared_volume')
        
        # Updated paths
        vocabulary_file_path = os.path.join(shared_volume_path, 
                                          "orb_slam3_ws/src/ORB-SLAM3-ROS2-Docker/ORB_SLAM3/Vocabulary/ORBvoc.txt")
        config_file_path = os.path.join(shared_volume_path, 
                                      "orb_slam3_ws/src/ORB-SLAM3-ROS2-Docker/orb_slam3_ros2_wrapper/params/gazebo_rgbd.yaml")
        
        # Check if files exist
        if not os.path.exists(vocabulary_file_path):
            print(f"WARNING: Vocabulary file not found at {vocabulary_file_path}")
        if not os.path.exists(config_file_path):
            print(f"WARNING: Config file not found at {config_file_path}")
            
        declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(orb_wrapper_pkg, 'params', 'dem-ros-params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes')

        # Get the namespace
        ns = robot_namespace.perform(context)
        
        # Create base frame with namespace
        base_frame = ""
        if(ns == ""):
            base_frame = ""
        else:
            base_frame = ns + "/"

        param_substitutions = {
            # 'robot_base_frame': base_frame + 'base_footprint',
            # 'odom_frame': base_frame + 'odom'
            }

        configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=ns,
            param_rewrites=param_substitutions,
            convert_types=True)
        
        # Define front stereo camera topics
        rgb_topic = f'/{ns}/front_stereo/left_cam/image_raw'
        depth_topic = f'/{ns}/front_stereo/right_cam/image_raw'
        camera_info_topic = f'/{ns}/front_stereo/left_cam/camera_info'
        
        # ORB-SLAM3 node with front stereo camera
        orb_slam3_node = Node(
            package='orb_slam3_ros2_wrapper',
            executable='rgbd',
            output='screen',
            namespace=ns,
            arguments=[vocabulary_file_path, config_file_path],
            parameters=[
                configured_params,
                {
                    'use_sim_time': True,
                    'robot_namespace': ns,
                    'robot_base_frame': f'{ns}/base_link',
                    'global_frame': 'map',
                    'odom_frame': f'{ns}/odom',
                    'visualization': False,
                    'odometry_mode': True,
                    'publish_tf': True,
                    'map_data_publish_frequency': 1000,
                    'do_loop_closing': True,
                    'rgb_image_topic_name': rgb_topic,
                    'depth_image_topic_name': depth_topic,
                    'camera_info_topic_name': camera_info_topic
                }
            ]
        )
        
        return [declare_params_file_cmd, orb_slam3_node]

    opaque_function = OpaqueFunction(function=all_nodes_launch, args=[robot_namespace])
#---------------------------------------------

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_namespace_arg,
        opaque_function
    ])
