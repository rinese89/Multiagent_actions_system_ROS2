import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    controller_dir = LaunchConfiguration(
        'controller_dir',
        default=os.path.join(
            get_package_share_directory('controller_pkg'), 'launch'))
    
    localization_dir = LaunchConfiguration(
        'localization_dir',
        default=os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch'))

    actions=[

        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([controller_dir, '/odom_pub_nodes_launch.py'])),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([localization_dir, '/bringup_launch.py']),
            launch_arguments={
                'namespace': 'tb3_0',
                'map': '/home/rinese/map_tfg_short.yaml',
                'use_namespace': 'true',
                'frame_id': 'map'
            }.items()),

        #ExecuteProcess(
        #    cmd=[['ros2 topic pub --once /tb3_0/initialpose geometry_msgs/msg/PoseWithCovarianceStamped """{header: {frame_id: """map"""}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},covariance: [0.01,0.0,0.0,0.0,0.0,0.0,0.0,0.01,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0076]} }"""']],
        #    shell=True
        #),

        #Node(
        #    package='controller_pkg',
        #    executable='odom_publish_tb31_node',
        #    name='odom_publish_tb31_node',
        #    output='screen',
        #),

        Node(
            package='controller_pkg',
            executable='reference_server_node',
            name='reference_server_node',
            output='screen',
        ),

        Node(
            package='controller_pkg',
            executable='control_leader_server_node',
            name='control_leader_server_node',
            output='screen',
        ),

        Node(
            package='controller_pkg',
            executable='control_follower_server_node',
            name='control_follower_server_node',
            output='screen',
        ),

        Node(
            package='controller_pkg',
            executable='control_lc_client_node',
            name='control_lc_client_node',
            output='screen',
        ),
    ]

    control_ref_cmd_group = GroupAction(actions)

    ld = LaunchDescription()
    ld.add_action(control_ref_cmd_group)

    return ld