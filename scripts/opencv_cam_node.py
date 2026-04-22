#!/usr/bin/env python3
"""
ROS 2 camera publisher using OpenCV — handles pixel formats that v4l2_camera
doesn't support (e.g. YU12/I420 from DroidCam / v4l2loopback devices).
Publishes sensor_msgs/Image on /<namespace>/image_raw with SensorDataQoS.
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class OpenCVCamNode(Node):
    def __init__(self):
        super().__init__('opencv_cam')

        self.declare_parameter('device',    '/dev/video8')
        self.declare_parameter('width',     640)
        self.declare_parameter('height',    480)
        self.declare_parameter('fps',       30)
        self.declare_parameter('frame_id',  'camera')

        device   = self.get_parameter('device').value
        width    = self.get_parameter('width').value
        height   = self.get_parameter('height').value
        fps      = self.get_parameter('fps').value
        frame_id = self.get_parameter('frame_id').value

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().fatal(f'Cannot open {device}')
            raise RuntimeError(f'Cannot open {device}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS,          fps)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'Opened {device}  {actual_w}x{actual_h} @ {actual_fps:.1f} fps')

        self.frame_id = frame_id
        self.bridge   = CvBridge()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub = self.create_publisher(Image, 'image_raw', qos)

        period = 1.0 / fps
        self.timer = self.create_timer(period, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Dropped frame — device not streaming?')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = OpenCVCamNode()
        rclpy.spin(node)
    except (RuntimeError, KeyboardInterrupt):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
