import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Device argument sentinel — empty string means "don't launch this device".
_SKIP = ''


def _uvc_node(device: str, index: int, pixel_format: str) -> Node:
    ns = f'cam_uvc_{index}'
    return Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name=ns,
        namespace=ns,
        parameters=[{
            'video_device':    device,
            'image_size':      [640, 480],
            'pixel_format':    pixel_format,
            'output_encoding': 'rgb8',
            'time_per_frame':  [1, 30],
        }],
        output='screen',
    )


def _rs_node(serial: str, index: int) -> Node:
    ns = f'camera_{index}'
    params = {
        'enable_color':               True,
        'enable_depth':               True,
        'enable_infra1':              False,
        'enable_infra2':              False,
        'enable_gyro':                False,
        'enable_accel':               False,
        'rgb_camera.color_profile':   '640x480x30',
        'depth_module.depth_profile': '640x480x30',
        'publish_tf':                 False,
    }
    if serial:
        params['serial_no'] = serial
    return Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace=ns,
        parameters=[params],
        output='screen',
    )


def launch_setup(context, *args, **kwargs):
    def get(name):
        return LaunchConfiguration(name).perform(context)

    uvc_0       = get('uvc_device_0')
    uvc_1       = get('uvc_device_1')
    uvc_fmt_0   = get('uvc_pixel_format_0')
    uvc_fmt_1   = get('uvc_pixel_format_1')
    rs_serial_0 = get('rs_serial_0')
    rs_serial_1 = get('rs_serial_1')
    slop_s      = float(get('slop_s'))
    csv_path    = get('csv_path')
    topics_raw  = get('topics')

    # Guard: two RealSense nodes without explicit serials will both
    # auto-discover and compete for the same physical device.
    if rs_serial_1 != _SKIP and rs_serial_0 == _SKIP:
        raise RuntimeError(
            'rs_serial_1 is set but rs_serial_0 is empty. '
            'Provide both serial numbers when using two RealSense devices: '
            'rs_serial_0:=<serial_A> rs_serial_1:=<serial_B>  '
            '(run: rs-enumerate-devices | grep "Serial Number")'
        )

    # Build node list — devices are skipped when their arg is empty.
    nodes = []
    auto_topics = []

    if uvc_0 != _SKIP:
        nodes.append(_uvc_node(uvc_0, 0, uvc_fmt_0))
        auto_topics.append('/cam_uvc_0/image_raw')

    # rs_serial_0 empty = auto-discover (safe only when one RealSense is connected).
    if rs_serial_0 != _SKIP:
        nodes.append(_rs_node(rs_serial_0, 0))
        auto_topics.append('/camera_0/camera/color/image_raw')

    if uvc_1 != _SKIP:
        nodes.append(_uvc_node(uvc_1, 1, uvc_fmt_1))
        auto_topics.append('/cam_uvc_1/image_raw')

    if rs_serial_1 != _SKIP:
        nodes.append(_rs_node(rs_serial_1, 1))
        auto_topics.append('/camera_1/camera/color/image_raw')

    # Use explicit topics arg if provided, otherwise derive from active devices.
    try:
        topics = yaml.safe_load(topics_raw)
        if not isinstance(topics, list) or not topics:
            raise ValueError
    except Exception:
        topics = auto_topics

    nodes.append(Node(
        package='cam_sync_bench',
        executable='sync_bench_node',
        name='sync_bench',
        parameters=[{
            'topics':          topics,
            'slop_s':          slop_s,
            'queue_size':      30,
            'print_period_s':  2.0,
            'csv_path':        csv_path,
        }],
        output='screen',
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # UVC cameras — pass empty string to skip the second one.
        DeclareLaunchArgument('uvc_device_0', default_value='/dev/video0',
                              description='V4L2 device path for UVC camera 0'),
        DeclareLaunchArgument('uvc_device_1', default_value=_SKIP,
                              description='V4L2 device path for UVC camera 1 (empty = skip)'),
        DeclareLaunchArgument('uvc_pixel_format_0', default_value='YUYV',
                              description='V4L2 pixel format for UVC camera 0'),
        DeclareLaunchArgument('uvc_pixel_format_1', default_value='YU12',
                              description='V4L2 pixel format for UVC camera 1 (YU12 for DroidCam/loopback)'),

        # RealSense cameras — always pass explicit serials when using two devices.
        # Single device: set rs_serial_0 only (auto-discover if you have exactly one RS).
        # Two devices: both serials required (run: rs-enumerate-devices | grep "Serial Number").
        DeclareLaunchArgument('rs_serial_0', default_value=_SKIP,
                              description='Serial number for RealSense 0 (empty = skip)'),
        DeclareLaunchArgument('rs_serial_1', default_value=_SKIP,
                              description='Serial number for RealSense 1 (empty = skip)'),

        # Benchmark parameters.
        DeclareLaunchArgument('slop_s',   default_value='0.034'),
        DeclareLaunchArgument('csv_path', default_value='/tmp/sync_bench.csv'),

        # Override topics explicitly, or leave empty to auto-derive from active devices.
        DeclareLaunchArgument('topics', default_value='',
                              description='YAML list of 2-4 topics; auto-built if empty'),

        OpaqueFunction(function=launch_setup),
    ])
