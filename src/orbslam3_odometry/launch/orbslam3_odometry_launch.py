from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    left_camera_info = os.path.join(
        get_package_share_directory('orbslam3_odometry'),
        'config', 'calibrations', 'clyde', 'in_water',
        'left_stereo_params.yaml'
        )

    right_camera_info = os.path.join(
        get_package_share_directory('orbslam3_odometry'),
        'config', 'calibrations', 'clyde', 'in_water',
        'left_stereo_params.yaml'
        )

    odometry_config = os.path.join(
        get_package_share_directory('orbslam3_odometry'),
        'config', 
        'orbslam3_odometry.yaml'
        )

    orbslam3_config = os.path.join(
        get_package_share_directory('orbslam3_odometry'),
        'config', 
        'aquarium_nano_stereo.yaml'
        )

    orbslam3_vocabulary = os.path.join(
        get_package_share_directory('orbslam3_odometry'),
        'config', 
        'ORBvoc.txt'
    )

    nodes= [
        Node(
            package='orbslam_helpers',
            name='left_camera_info_injector',
            executable='camera_info_injector',
            parameters=[{
                'camera_info_file': left_camera_info
            }],
            remappings=[
                ('input/image_raw', '/left/image_raw'),
                ('output/camera_info', '/left/output/camera_info'),
                ('output/image_raw', '/left/output/image_raw')
            ],
        ),
        Node(
            package='image_proc',
            name='rectify_node',
            executable='rectify_node',
            remappings=[
                ('image',       '/left/output/image_raw'),
                ('camera_info', '/left/output/camera_info'),
                ('image_rect',  '/left/image_rect')
            ],
        ),

        Node(
            package='orbslam_helpers',
            name='right_camera_info_injector',
            executable='camera_info_injector',
            parameters=[{
                'camera_info_file': right_camera_info
            }],
            remappings=[
                ('input/image_raw', '/right/image_raw'),
                ('output/camera_info', '/right/output/camera_info'),
                ('output/image_raw', '/right/output/image_raw')
            ],
        ),
        Node(
            package='image_proc',
            name='rectify_node',
            executable='rectify_node',
            remappings=[
                ('image',       '/right/output/image_raw'),
                ('camera_info', '/right/output/camera_info'),
                ('image_rect',  '/right/image_rect')
            ],
        ),

        Node(
            package='orbslam3_odometry',
            name='orbslam3_odometry',
            executable='orbslam3_odometry',
            output = 'screen',
            parameters=[odometry_config,
                        {
                            'path_yaml_settings': orbslam3_config,
                            'path_vocabulary': orbslam3_vocabulary
                        }]
        ),
    ]

    return LaunchDescription(nodes)