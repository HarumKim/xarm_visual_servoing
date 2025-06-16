import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_vision = get_package_share_directory('xarm_vision')
    config_file = os.path.join(pkg_vision, 'config', 'segmentation_point_cloud_config.yaml')
    intrinsics_file = os.path.join(pkg_vision, 'config', 'kinect_calibration.yaml')
    
    # Launch arguments for pose estimation
    launch_args = [
        DeclareLaunchArgument(
            'reference_ply_file',
            default_value='/home/kim/Downloads/OilPan/PointClouds/oil_pan_full_pc_10000.ply',
            description='Path to reference .ply file'
        ),
        DeclareLaunchArgument(
            'update_rate',
            default_value='10.0',
            description='Update rate in Hz'
        ),
        DeclareLaunchArgument(
            'voxel_size',
            default_value='0.002',
            description='Voxel size for downsampling'
        ),
        DeclareLaunchArgument(
            'icp_threshold',
            default_value='0.02',
            description='ICP distance threshold'
        ),
        DeclareLaunchArgument(
            'depth_scale',
            default_value='1000.0',
            description='Depth scale factor'
        ),
    ]
    
    image_segmentation_node = Node(
        package='xarm_vision',
        executable='image_segmentation',
        name='image_segmentation',
        parameters=[config_file],
        output='screen'
    )
    
    # Tu nodo actual que hace point cloud generation + pose estimation
    point_cloud_generator_node = Node(
        package='xarm_vision',
        executable='point_cloud_generator',  # Este es tu archivo actual
        name='point_cloud_generator',
        parameters=[
            config_file,
            {
                'camera_intrinsics_file': intrinsics_file,
                'reference_ply_file': LaunchConfiguration('reference_ply_file'),
                'update_rate': LaunchConfiguration('update_rate'),
                'voxel_size': LaunchConfiguration('voxel_size'),
                'icp_threshold': LaunchConfiguration('icp_threshold'),
                'depth_scale': LaunchConfiguration('depth_scale'),
                'icp_max_iterations': 50,
                # Camera transform parameters
                'tf_x': -0.236,
                'tf_y': -0.359,
                'tf_z': 0.1235,
                'tf_qx': -0.7068252,
                'tf_qy': 0.0,
                'tf_qz': 0.0,
                'tf_qw': 0.7073883,
            }
        ],
        output='screen'
    )

    pose_estimator_node = Node(
        package='xarm_vision',
        executable='pose_estimator',  # Este es tu archivo actual
        name='pose_estimator',
        parameters=[
            config_file,
            {
                'camera_intrinsics_file': intrinsics_file,
                'reference_ply_file': LaunchConfiguration('reference_ply_file'),
                'update_rate': LaunchConfiguration('update_rate'),
                'voxel_size': LaunchConfiguration('voxel_size'),
                'icp_threshold': LaunchConfiguration('icp_threshold'),
                'depth_scale': LaunchConfiguration('depth_scale'),
                'icp_max_iterations': 50,
            }
        ],
        output='screen'
    )

    xarm_kinect_node = Node(
        package='xarm_vision',
        executable='xarm_kinect',  # Este es tu archivo actual
        name='xarm_kinect',
        parameters=[
            config_file,
            {
                'camera_intrinsics_file': intrinsics_file,
                'reference_ply_file': LaunchConfiguration('reference_ply_file'),
                'update_rate': LaunchConfiguration('update_rate'),
                'voxel_size': LaunchConfiguration('voxel_size'),
                'icp_threshold': LaunchConfiguration('icp_threshold'),
                'depth_scale': LaunchConfiguration('depth_scale'),
                'icp_max_iterations': 50,
            }
        ],
        output='screen'
    )

    return LaunchDescription(
        launch_args + [
            image_segmentation_node,
            point_cloud_generator_node,
            pose_estimator_node,
            xarm_kinect_node
        ]
    )