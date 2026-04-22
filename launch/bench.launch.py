import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Device argument sentinel — empty string means "don't launch this device".
_SKIP = ''

# Absolute path to the opencv_cam_node script (resolved at import time so it
# works regardless of the working directory at launch).
_OPENCV_CAM_SCRIPT = os.path.join(
    os.path.dirname(__file__), '..', 'scripts', 'opencv_cam_node.py'
)


def _uvc_node(device: str, index: int, pixel_format: str, use_opencv: bool) -> Node:
    ns = f'cam_uvc_{index}'
    if use_opencv:
        # opencv_cam_node handles YU12/I420 and other formats v4l2_camera rejects.
        return Node(
            package='cam_sync_bench',
            executable='opencv_cam_node',
            name=ns,
            namespace=ns,
            parameters=[{
                'device':    device,
                'width':     640,
                'height':    480,
                'fps':       30,
                'frame_id':  'camera',
            }],
            output='screen',
        )
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


def _rs_node(serial: str, index: int, enable_depth: bool) -> Node:
    ns = f'camera_{index}'
    params = {
        'enable_color':               True,
        'enable_depth':               enable_depth,
        'enable_infra1':              False,
        'enable_infra2':              False,
        'enable_gyro':                False,
        'enable_accel':               False,
        'rgb_camera.color_profile':   '640x480x30',
        'publish_tf':                 False,
        # Align RealSense timestamps to system clock (CLOCK_REALTIME) so they
        # are in the same domain as V4L2/UVC timestamps. Without this the SDK
        # uses the camera's hardware clock, making cross-camera deltas meaningless.
        'enable_global_time_sync':    True,
    }
    if enable_depth:
        params['depth_module.depth_profile'] = '640x480x30'
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


def _str_to_bool(s: str) -> bool:
    return s.lower() in ('true', '1', 'yes')


def launch_setup(context, *args, **kwargs):
    def get(name):
        return LaunchConfiguration(name).perform(context)

    uvc_0          = get('uvc_device_0')
    uvc_1          = get('uvc_device_1')
    uvc_fmt_0      = get('uvc_pixel_format_0')
    uvc_fmt_1      = get('uvc_pixel_format_1')
    uvc_opencv_0   = _str_to_bool(get('uvc_use_opencv_0'))
    uvc_opencv_1   = _str_to_bool(get('uvc_use_opencv_1'))
    rs_serial_0    = get('rs_serial_0')
    rs_serial_1    = get('rs_serial_1')
    rs_depth_0     = _str_to_bool(get('rs_enable_depth_0'))
    rs_depth_1     = _str_to_bool(get('rs_enable_depth_1'))
    slop_s         = float(get('slop_s'))
    csv_path       = get('csv_path')
    topics_raw     = get('topics')
    sync_delay_s   = float(get('sync_delay_s'))

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
        nodes.append(_uvc_node(uvc_0, 0, uvc_fmt_0, uvc_opencv_0))
        auto_topics.append('/cam_uvc_0/image_raw')

    # # rs_serial_0 empty = auto-discover (safe only when one RealSense is connected).
    if rs_serial_0 != _SKIP:
        nodes.append(_rs_node(rs_serial_0, 0, rs_depth_0))
        auto_topics.append('/camera_0/camera/color/image_raw')

    if uvc_1 != _SKIP:
        nodes.append(_uvc_node(uvc_1, 1, uvc_fmt_1, uvc_opencv_1))
        auto_topics.append('/cam_uvc_1/image_raw')

    if rs_serial_1 != _SKIP:
        nodes.append(_rs_node(rs_serial_1, 1, rs_depth_1))
        auto_topics.append('/camera_1/camera/color/image_raw')

    # Use explicit topics arg if provided, otherwise derive from active devices.
    try:
        topics = yaml.safe_load(topics_raw)
        if not isinstance(topics, list) or not topics:
            raise ValueError
    except Exception:
        topics = auto_topics

    sync_node = Node(
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
    )

    # Delay sync_bench_node so cameras have time to initialise before it starts
    # trying to match frames. Without this, the synchronizer sees partial startup
    # traffic and its internal queues fill with unmatched frames.
    if sync_delay_s > 0.0:
        nodes.append(TimerAction(period=sync_delay_s, actions=[sync_node]))
    else:
        nodes.append(sync_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # UVC cameras — pass empty string to skip.
        DeclareLaunchArgument('uvc_device_0', default_value='/dev/video0',
                              description='V4L2 device path for UVC camera 0'),
        DeclareLaunchArgument('uvc_device_1', default_value=_SKIP,
                              description='V4L2 device path for UVC camera 1 (empty = skip)'),
        DeclareLaunchArgument('uvc_pixel_format_0', default_value='YUYV',
                              description='V4L2 pixel format for UVC camera 0 (ignored when use_opencv=true)'),
        DeclareLaunchArgument('uvc_pixel_format_1', default_value='YU12',
                              description='V4L2 pixel format for UVC camera 1 (ignored when use_opencv=true)'),
        # Use opencv_cam_node instead of v4l2_camera_node (required for YU12/I420 devices).
        DeclareLaunchArgument('uvc_use_opencv_0', default_value='false',
                              description='Use opencv_cam_node for UVC camera 0 (true for YU12/I420 sources)'),
        DeclareLaunchArgument('uvc_use_opencv_1', default_value='true',
                              description='Use opencv_cam_node for UVC camera 1 (true for DroidCam/loopback)'),

        # RealSense cameras — always pass explicit serials when using two devices.
        # Single device: set rs_serial_0 only (auto-discover if you have exactly one RS).
        # Two devices: both serials required (run: rs-enumerate-devices | grep "Serial Number").
        DeclareLaunchArgument('rs_serial_0', default_value=_SKIP,
                              description='Serial number for RealSense 0 (empty = skip)'),
        DeclareLaunchArgument('rs_serial_1', default_value=_SKIP,
                              description='Serial number for RealSense 1 (empty = skip)'),
        # Disable depth on USB 2.1 devices — color + depth at 640x480x30 exceeds USB 2.1 bandwidth.
        DeclareLaunchArgument('rs_enable_depth_0', default_value='true',
                              description='Enable depth stream for RealSense 0'),
        DeclareLaunchArgument('rs_enable_depth_1', default_value='true',
                              description='Enable depth stream for RealSense 1 (set false for USB 2.1 devices)'),

        # Benchmark parameters.
        DeclareLaunchArgument('slop_s',   default_value='0.034'),
        DeclareLaunchArgument('csv_path', default_value='/tmp/sync_bench.csv'),

        # Delay before sync_bench_node starts — gives cameras time to initialise.
        DeclareLaunchArgument('sync_delay_s', default_value='10.0',
                              description='Seconds to wait before starting sync_bench_node (0 = immediate)'),

        # Override topics explicitly, or leave empty to auto-derive from active devices.
        DeclareLaunchArgument('topics', default_value='',
                              description='YAML list of 2-4 topics; auto-built if empty'),

        OpaqueFunction(function=launch_setup),
    ])
