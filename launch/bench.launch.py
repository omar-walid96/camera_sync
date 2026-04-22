from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    uvc_device = LaunchConfiguration('uvc_device')
    width      = LaunchConfiguration('width')
    height     = LaunchConfiguration('height')
    fps        = LaunchConfiguration('fps')
    slop_s     = LaunchConfiguration('slop_s')
    csv_path   = LaunchConfiguration('csv_path')

    return LaunchDescription([
        DeclareLaunchArgument('uvc_device', default_value='/dev/video0'),
        DeclareLaunchArgument('width',  default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps',    default_value='30'),
        DeclareLaunchArgument('slop_s', default_value='0.034'),
        DeclareLaunchArgument('csv_path', default_value='/tmp/sync_bench.csv'),

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='cam_uvc',
            namespace='cam_uvc',
            parameters=[{
                'video_device': uvc_device,
                'image_size': [640, 480],
                'pixel_format': 'YUYV',
                'output_encoding': 'rgb8',
                'time_per_frame': [1, 30],
            }],
            output='screen',
        ),

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': False,
                'enable_accel': False,
                'rgb_camera.color_profile': '640x480x30',
                'depth_module.depth_profile': '640x480x30',
                'publish_tf': False,
            }],
            output='screen',
        ),

        Node(
            package='cam_sync_bench',
            executable='sync_bench_node',
            name='sync_bench',
            parameters=[{
                'topic_a': '/cam_uvc/image_raw',
                'topic_b': '/camera/camera/color/image_raw',
                'slop_s': slop_s,
                'queue_size': 30,
                'print_period_s': 2.0,
                'csv_path': csv_path,
            }],
            output='screen',
        ),
    ])
