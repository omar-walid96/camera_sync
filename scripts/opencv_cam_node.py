#!/usr/bin/env python3
"""
ROS 2 camera publisher using OpenCV — handles pixel formats that v4l2_camera
doesn't support (e.g. YU12/I420 from DroidCam / v4l2loopback devices).
Publishes sensor_msgs/Image on /<namespace>/image_raw with SensorDataQoS.

Capture runs on the main thread in a tight blocking loop (identical to
test_uvc.py). rclpy.spin() is intentionally not called — this node has no
subscriptions or timers, so spin() only adds GIL contention that halves fps.

Deliberately avoids cv_bridge: the Humble cv_bridge Python binding was compiled
against NumPy 1.x and raises AttributeError on NumPy 2.x systems.
"""
import array as _array
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
from sensor_msgs.msg import Image


def _bgr_to_imgmsg(frame, stamp, frame_id) -> Image:
    h, w = frame.shape[:2]
    msg = Image()
    msg.header.stamp    = stamp
    msg.header.frame_id = frame_id
    msg.height          = h
    msg.width           = w
    msg.encoding        = 'bgr8'
    msg.is_bigendian    = False
    msg.step            = w * 3
    # array.array('B', bytes) uses a C-level memcpy fast path in the rclpy
    # uint8[] setter. Passing plain bytes triggers a Python-level per-element
    # validation loop over all 921,600 bytes (~50 ms at 640x480), halving fps.
    msg.data            = _array.array('B', frame.tobytes())
    return msg


class OpenCVCamNode(Node):
    def __init__(self):
        super().__init__('opencv_cam')

        self.declare_parameter('device',   '/dev/video8')
        self.declare_parameter('width',    640)
        self.declare_parameter('height',   480)
        self.declare_parameter('fps',      30)
        self.declare_parameter('frame_id', 'camera')

        device   = self.get_parameter('device').value
        width    = self.get_parameter('width').value
        height   = self.get_parameter('height').value
        fps      = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().fatal(f'Cannot open {device}')
            raise RuntimeError(f'Cannot open {device}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS,          fps)

        actual_w   = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'Opened {device}  {actual_w}x{actual_h} @ {actual_fps:.1f} fps')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub = self.create_publisher(Image, 'image_raw', qos)

    def run(self):
        """Tight blocking capture loop — runs on the main thread, no GIL competition."""
        import time
        count = 0
        while rclpy.ok():
            t0 = time.monotonic()
            ret, frame = self.cap.read()
            t1 = time.monotonic()
            if not ret:
                self.get_logger().warn('Dropped frame — device not streaming?')
                continue
            msg = _bgr_to_imgmsg(
                frame, self.get_clock().now().to_msg(), self.frame_id)
            t2 = time.monotonic()
            self.pub.publish(msg)
            t3 = time.monotonic()
            count += 1
            if count % 60 == 0:
                self.get_logger().info(
                    f'read={1000*(t1-t0):.1f}ms '
                    f'conv={1000*(t2-t1):.1f}ms '
                    f'pub={1000*(t3-t2):.1f}ms '
                    f'total={1000*(t3-t0):.1f}ms'
                )

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = OpenCVCamNode()
        node.run()
    except (RuntimeError, KeyboardInterrupt):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
