#!/usr/bin/env python3
"""
detector_node.py
────────────────────────────────────────────────────────────────────────────
Subscribes to RGB + depth images from the OAK-D (sim or hardware).
Runs YOLOv8 detection, back-projects bbox centers to 3D using the depth
image + camera intrinsics, transforms to base_link via TF2, and publishes
WasteDetectionArray on /cika/waste_detections.

classifier_node.py subscribes downstream to enrich the label confidence.

Sim backend  : ultralytics YOLOv8 .pt — CPU or CUDA
Hardware     : .blob on OAK-D VPU via depthai (swap backend in config)
"""

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — required for tf2 PointStamped transform

from cika_perception.msg import WasteDetection, WasteDetectionArray
from cika_core.utils.depth_projection import (
    bbox_center,
    sample_depth,
    deproject_pixel,
    make_point_stamped,
)
from cika_core.utils.visualization import draw_detections


# YOLO class id → label — matches taco_binary_kaggle.yaml training config
CLASS_NAMES = {0: "recyclable", 1: "non_recyclable"}

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=5,
)


class DetectorNode(Node):

    def __init__(self):
        super().__init__("detector_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence_threshold", 0.45)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("rgb_topic", "/oak/rgb/image_raw")
        self.declare_parameter("depth_topic", "/oak/stereo/image_raw")
        self.declare_parameter("camera_info_topic", "/oak/rgb/camera_info")
        self.declare_parameter("detections_topic", "/cika/waste_detections")
        self.declare_parameter("debug_image_topic", "/cika/perception/debug_image")
        self.declare_parameter("camera_frame", "OAKDcamera_1_optical")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("inference_rate", 5.0)

        self.conf_thresh   = self.get_parameter("confidence_threshold").value
        self.iou_thresh    = self.get_parameter("iou_threshold").value
        self.device        = self.get_parameter("device").value
        self.camera_frame  = self.get_parameter("camera_frame").value
        self.base_frame    = self.get_parameter("base_frame").value
        model_path         = self.get_parameter("model_path").value

        # ── YOLO model ────────────────────────────────────────────────────────
        self.model = self._load_model(model_path)

        # ── Camera intrinsics (populated on first CameraInfo msg) ─────────────
        self.camera_model = PinholeCameraModel()
        self._camera_info_ready = False

        # ── TF2 ───────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── CV bridge ─────────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Rate limiter ──────────────────────────────────────────────────────
        rate = self.get_parameter("inference_rate").value
        self._inference_period = 1.0 / max(rate, 0.1)
        self._last_inference_t = 0.0

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_cb,
            SENSOR_QOS,
        )

        rgb_sub   = message_filters.Subscriber(self, Image, self.get_parameter("rgb_topic").value,   qos_profile=SENSOR_QOS)
        depth_sub = message_filters.Subscriber(self, Image, self.get_parameter("depth_topic").value, qos_profile=SENSOR_QOS)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self._image_cb)

        # ── Publishers ────────────────────────────────────────────────────────
        self.detections_pub = self.create_publisher(
            WasteDetectionArray,
            self.get_parameter("detections_topic").value,
            10,
        )
        self.debug_pub = self.create_publisher(
            Image,
            self.get_parameter("debug_image_topic").value,
            10,
        )

        self.get_logger().info(
            f"DetectorNode ready | model: {model_path} | device: {self.device} "
            f"| conf: {self.conf_thresh} | rate: {rate} Hz"
        )

    # ── Model loading ──────────────────────────────────────────────────────────
    def _load_model(self, model_path: str):
        if not model_path:
            self.get_logger().error("model_path is empty — check perception_sim.yaml")
            return None
        try:
            from ultralytics import YOLO
            model = YOLO(model_path)
            self.get_logger().info(f"Loaded YOLO model: {model_path}")
            return model
        except Exception as exc:
            self.get_logger().error(f"YOLO load failed: {exc}")
            return None

    # ── Camera info ────────────────────────────────────────────────────────────
    def _camera_info_cb(self, msg: CameraInfo):
        if not self._camera_info_ready:
            self.camera_model.fromCameraInfo(msg)
            self._camera_info_ready = True
            self.get_logger().info("Camera intrinsics received — 3D projection enabled")

    # ── Main synchronized callback ─────────────────────────────────────────────
    def _image_cb(self, rgb_msg: Image, depth_msg: Image):
        if self.model is None:
            return

        # Rate limit
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self._last_inference_t) < self._inference_period:
            return
        self._last_inference_t = now

        try:
            rgb   = self.bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(f"Image conversion error: {exc}")
            return

        # ── YOLO inference ────────────────────────────────────────────────────
        results = self.model.predict(
            rgb,
            conf=self.conf_thresh,
            iou=self.iou_thresh,
            device=self.device,
            verbose=False,
        )

        det_array = WasteDetectionArray()
        det_array.header = rgb_msg.header
        viz_list = []

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                det = self._process_box(box, depth, rgb_msg.header.stamp)
                if det is None:
                    continue
                det_array.detections.append(det)
                viz_list.append({
                    "bbox":                   det.bbox,
                    "label":                  det.label,
                    "confidence_detector":    det.confidence_detector,
                    "confidence_classifier":  det.confidence_classifier,
                    "has_3d_position":        det.has_3d_position,
                    "position": (det.position.x, det.position.y, det.position.z)
                                if det.has_3d_position else None,
                })

        self.detections_pub.publish(det_array)

        # Only render debug image if someone is subscribed (saves CPU)
        if self.debug_pub.get_subscription_count() > 0:
            debug_img = draw_detections(rgb, viz_list)
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            )

    # ── Process a single YOLO box ──────────────────────────────────────────────
    def _process_box(self, box, depth: np.ndarray, stamp) -> WasteDetection | None:
        try:
            xyxy   = box.xyxy[0].cpu().numpy()
            conf   = float(box.conf[0].cpu().numpy())
            cls_id = int(box.cls[0].cpu().numpy())
        except Exception as exc:
            self.get_logger().warn(f"Box parse error: {exc}")
            return None

        det = WasteDetection()
        det.header.stamp    = stamp
        det.header.frame_id = self.base_frame
        det.bbox                 = [float(v) for v in xyxy]
        det.confidence_detector  = conf
        det.label                = CLASS_NAMES.get(cls_id, "unknown")
        det.confidence_classifier = 0.0   # filled by classifier_node
        det.has_3d_position      = False

        # ── 3D projection ──────────────────────────────────────────────────────
        if self._camera_info_ready and depth is not None:
            u, v     = bbox_center(det.bbox)
            depth_m  = sample_depth(depth, u, v)
            xyz_cam  = deproject_pixel(self.camera_model, u, v, depth_m)

            if xyz_cam is not None:
                ps_cam = make_point_stamped(xyz_cam, self.camera_frame, stamp)
                try:
                    ps_base = self.tf_buffer.transform(
                        ps_cam,
                        self.base_frame,
                        timeout=rclpy.duration.Duration(seconds=0.1),
                    )
                    det.position.x      = ps_base.point.x
                    det.position.y      = ps_base.point.y
                    det.position.z      = ps_base.point.z
                    det.has_3d_position = True
                except Exception as exc:
                    self.get_logger().warn(
                        f"TF transform failed: {exc}", throttle_duration_sec=5.0
                    )

        return det


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()