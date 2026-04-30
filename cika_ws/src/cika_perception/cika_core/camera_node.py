#!/usr/bin/env python3
"""
camera_node.py
────────────────────────────────────────────────────────────────────────────
Publishes OAK-D Lite RGB + stereo depth as ROS2 Image topics using the
depthai Python SDK. Replaces ros-humble-depthai-ros for arm64 Pi deployment.

Publishes:
    /oak/rgb/image_raw        (sensor_msgs/Image, BGR8)
    /oak/stereo/image_raw     (sensor_msgs/Image, 16UC1 depth in mm)
    /oak/rgb/camera_info      (sensor_msgs/CameraInfo)
    /oak/stereo/camera_info   (sensor_msgs/CameraInfo)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import depthai as dai
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from builtin_interfaces.msg import Time


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=5,
)


class CameraNode(Node):

    def __init__(self):
        super().__init__("camera_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("rgb_width",    1280)
        self.declare_parameter("rgb_height",   720)
        self.declare_parameter("fps",          15)
        self.declare_parameter("camera_frame", "OAKDcamera_1_optical")

        self.rgb_w        = self.get_parameter("rgb_width").value
        self.rgb_h        = self.get_parameter("rgb_height").value
        self.fps          = self.get_parameter("fps").value
        self.camera_frame = self.get_parameter("camera_frame").value

        # ── Publishers ────────────────────────────────────────────────────────
        self.rgb_pub        = self.create_publisher(Image,      "/oak/rgb/image_raw",      SENSOR_QOS)
        self.depth_pub      = self.create_publisher(Image,      "/oak/stereo/image_raw",   SENSOR_QOS)
        self.rgb_info_pub   = self.create_publisher(CameraInfo, "/oak/rgb/camera_info",    SENSOR_QOS)
        self.depth_info_pub = self.create_publisher(CameraInfo, "/oak/stereo/camera_info", SENSOR_QOS)

        self.bridge = CvBridge()

        # ── Build depthai pipeline ────────────────────────────────────────────
        self.pipeline = self._build_pipeline()
        self.device   = dai.Device(self.pipeline)

        self.rgb_queue   = self.device.getOutputQueue("rgb",   maxSize=4, blocking=False)
        self.depth_queue = self.device.getOutputQueue("depth", maxSize=4, blocking=False)

        # ── Timer — poll camera at fps rate ───────────────────────────────────
        self.create_timer(1.0 / self.fps, self._timer_cb)

        self.get_logger().info(
            f"CameraNode ready | {self.rgb_w}x{self.rgb_h} @ {self.fps}fps "
            f"| frame: {self.camera_frame}"
        )

    # ── Pipeline ───────────────────────────────────────────────────────────────
    def _build_pipeline(self):
        pipeline = dai.Pipeline()

        # RGB camera
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setPreviewSize(self.rgb_w, self.rgb_h)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(self.fps)

        # Stereo depth
        mono_left  = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        stereo     = pipeline.create(dai.node.StereoDepth)

        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_left.setFps(self.fps)

        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        mono_right.setFps(self.fps)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.FAST_ACCURACY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # align depth to RGB
        stereo.setOutputSize(self.rgb_w, self.rgb_h)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # Output streams
        xout_rgb   = pipeline.create(dai.node.XLinkOut)
        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        xout_depth.setStreamName("depth")

        cam_rgb.preview.link(xout_rgb.input)
        stereo.depth.link(xout_depth.input)

        return pipeline

    # ── Timer callback ─────────────────────────────────────────────────────────
    def _timer_cb(self):
        now = self.get_clock().now().to_msg()

        rgb_frame   = self.rgb_queue.tryGet()
        depth_frame = self.depth_queue.tryGet()

        if rgb_frame is not None:
            rgb_img = rgb_frame.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(rgb_img, encoding="bgr8")
            msg.header.stamp    = now
            msg.header.frame_id = self.camera_frame
            self.rgb_pub.publish(msg)
            self.rgb_info_pub.publish(self._make_camera_info(now, self.rgb_w, self.rgb_h))

        if depth_frame is not None:
            depth_img = depth_frame.getCvFrame().astype(np.uint16)
            msg = self.bridge.cv2_to_imgmsg(depth_img, encoding="16UC1")
            msg.header.stamp    = now
            msg.header.frame_id = self.camera_frame
            self.depth_pub.publish(msg)
            self.depth_info_pub.publish(self._make_camera_info(now, self.rgb_w, self.rgb_h))

    # ── Camera info ────────────────────────────────────────────────────────────
    def _make_camera_info(self, stamp: Time, width: int, height: int) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.camera_frame
        msg.width  = width
        msg.height = height
        # Approximate intrinsics for OAK-D Lite at 1280x720
        # Replace with actual calibration values from oakd_calibration if available
        fx = 857.5
        fy = 857.5
        cx = width  / 2.0
        cy = height / 2.0
        msg.k = [fx, 0.0, cx,
                 0.0, fy, cy,
                 0.0, 0.0, 1.0]
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0,
                 0.0, fy, cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        msg.distortion_model = "plumb_bob"
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()