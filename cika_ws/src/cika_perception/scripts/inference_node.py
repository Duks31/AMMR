#!/usr/bin/env python3
"""
ROS2 node for running inference on OAK-D Lite with DepthAI v3 API.
Host-Side Decoding + Host-Side NumPy Spatial Calculation (Crash-Free)
"""

import os
import numpy as np
import rclpy
import cv2
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from rclpy.node import Node
import depthai as dai
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool
from ament_index_python.packages import get_package_share_directory

from cika_perception.msg import WasteDetectionArray, WasteDetection
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

package_share_directory = get_package_share_directory("cika_perception")

# ── Model paths ───────────────────────────────────────────────────────────────
BLOB_PATH = os.path.join(package_share_directory, "models", "taco_2class_300epoch.blob")

# ── Detection config ──────────────────────────────────────────────────────────
INPUT_W = 640
INPUT_H = 640
CONF_THRESH = 0.5
IOU_THRESH = 0.5
CLASSES = ["plastic", "paper"]
NC = len(CLASSES)

# ── Depth thresholds in mm ────────────────────────────────────────────────────
DEPTH_MIN_MM = 100
DEPTH_MAX_MM = 8000

# ── Hardware profile ──────────────────────────────────────────────────────────
ON_PI = True
CAM_FPS = 10 if ON_PI else 30
LRC_ENABLED = True

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1
)


# ── YOLO Parsing Utilities ────────────────────────────────────────────────────
def xywh2xyxy(cx, cy, w, h):
    return cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2


def nms(boxes, scores, iou_thresh):
    if len(boxes) == 0:
        return []
    boxes = np.array(boxes, dtype=np.float32)
    scores = np.array(scores, dtype=np.float32)
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w_ = np.maximum(0.0, xx2 - xx1)
        h_ = np.maximum(0.0, yy2 - yy1)
        iou = (w_ * h_) / (areas[i] + areas[order[1:]] - w_ * h_ + 1e-6)
        order = order[np.where(iou <= iou_thresh)[0] + 1]
    return keep


def parse_yolov8(output: np.ndarray, conf_thresh: float, iou_thresh: float):
    pred = output[0].T  # [8400, 4+NC]
    boxes, scores, class_ids = [], [], []
    for row in pred:
        cx, cy, w, h = row[0], row[1], row[2], row[3]
        class_scores = row[4:]
        class_id = int(np.argmax(class_scores))
        confidence = float(class_scores[class_id])
        if confidence < conf_thresh:
            continue
        x1, y1, x2, y2 = xywh2xyxy(cx, cy, w, h)
        boxes.append([x1, y1, x2, y2])
        scores.append(confidence)
        class_ids.append(class_id)
    keep = nms(boxes, scores, iou_thresh)
    return [(boxes[i], scores[i], class_ids[i]) for i in keep]


# ── ROS2 Node ─────────────────────────────────────────────────────────────────
class InferenceNode(Node):
    def __init__(self):
        super().__init__("inference_node")

        self.publisher = self.create_publisher(
            WasteDetectionArray, "/cika/perception/waste_detections", 10
        )
        self.image_pub = self.create_publisher(
            CompressedImage, "/cika/perception/image_raw/compressed", qos
        )

        self.rgb_raw_pub   = self.create_publisher(Image, '/oak/rgb/image_raw', qos)
        self.depth_raw_pub = self.create_publisher(Image, '/oak/stereo/image_raw', qos)
        self.cam_info_pub  = self.create_publisher(CameraInfo, '/oak/rgb/camera_info', qos)

        self._pipeline = None
        self._nn_queue = None
        self._img_queue = None
        self._depth_queue = None

        self._inference_active = False
        self._latest_detections = []

        self._start_pipeline()

        calib = self._pipeline.getDefaultDevice().readCalibration()
        M = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, INPUT_W, INPUT_H)
        self._fx = M[0][0]
        self._fy = M[1][1]
        self._cx = M[0][2]
        self._cy = M[1][2]
        self._cam_matrix = M

        self.create_timer(0.1, self._poll)
        self.create_timer(0.1, self._publish_image)
        self.create_timer(0.1, self._publish_raw_topics)
        self.create_service(
            SetBool, "/cika/perception/set_inference_active", self._enable_cb
        )
        self.get_logger().info(
            "Inference Node Ready. Host-Side NumPy Spatial Calc Active."
        )

    def _enable_cb(self, request, response):
        self._inference_active = request.data
        self.get_logger().info(f"Inference {'enabled' if request.data else 'disabled'}")
        response.success = True
        response.message = "ok"
        return response

    def _start_pipeline(self):
        pipeline = dai.Pipeline()

        # ── RGB camera ────────────────────────────────────────────────────────
        cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        cam_nn_out = cam_rgb.requestOutput(
            (INPUT_W, INPUT_H), type=dai.ImgFrame.Type.BGR888p, fps=CAM_FPS
        )
        cam_video_out = cam_rgb.requestOutput(
            (INPUT_W, INPUT_H), type=dai.ImgFrame.Type.BGR888p, fps=CAM_FPS
        )

        # ── Mono cameras ──────────────────────────────────────────────────────
        mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        mono_left_out = mono_left.requestOutput((640, 400), fps=CAM_FPS)
        mono_right_out = mono_right.requestOutput((640, 400), fps=CAM_FPS)

        # ── StereoDepth ───────────────────────────────────────────────────────
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.FAST_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(INPUT_W, INPUT_H)
        stereo.setLeftRightCheck(LRC_ENABLED)
        stereo.setSubpixel(False)
        stereo.setRectification(True)
        mono_left_out.link(stereo.left)
        mono_right_out.link(stereo.right)

        # ── Basic NeuralNetwork Node (Will NOT Crash) ─────────────────────────
        nn = pipeline.create(dai.node.NeuralNetwork)
        nn.setBlobPath(BLOB_PATH)
        nn.setNumInferenceThreads(2)
        cam_nn_out.link(nn.input)

        # ── Output queues ─────────────────────────────────────────────────────
        self._nn_queue = nn.out.createOutputQueue(maxSize=4, blocking=False)
        self._img_queue = cam_video_out.createOutputQueue(maxSize=1, blocking=False)
        self._depth_queue = stereo.depth.createOutputQueue(maxSize=4, blocking=False)
        self._raw_depth_queue = stereo.depth.createOutputQueue(maxSize=1, blocking=False)

        pipeline.start()
        self._pipeline = pipeline

    def _get_spatial_coordinates(self, box, depth_frame):
        """Calculate 3D position using Numpy to crop the depth map."""
        if depth_frame is None:
            return None

        x1, y1, x2, y2 = map(int, box)

        # Ensure the box stays inside the 640x640 frame
        x1 = max(0, min(x1, INPUT_W - 1))
        y1 = max(0, min(y1, INPUT_H - 1))
        x2 = max(0, min(x2, INPUT_W - 1))
        y2 = max(0, min(y2, INPUT_H - 1))

        if x1 >= x2 or y1 >= y2:
            return None

        # Crop the depth map exactly where the bounding box is
        roi_depth = depth_frame[y1:y2, x1:x2]

        # Ignore pixels that are too close (0) or too far
        valid_depths = roi_depth[
            (roi_depth > DEPTH_MIN_MM) & (roi_depth < DEPTH_MAX_MM)
        ]

        if len(valid_depths) == 0:
            return None

        # Calculate the median depth (Z axis)
        z_mm = float(np.median(valid_depths))
        z_m = z_mm / 1000.0

        # Calculate X and Y using standard Pinhole Camera Math
        # OAK-D Lite Focal Length at 640px is ~432.4
        # focal_length = 432.4
        # cx = (x1 + x2) / 2.0
        # cy = (y1 + y2) / 2.0

        # x_m = (cx - (INPUT_W / 2.0)) * z_m / focal_length
        # y_m = (cy - (INPUT_H / 2.0)) * z_m / focal_length

        px = (x1 + x2) / 2.0
        py = (y1 + y2) / 2.0
        x_m = (px - self._cx) * z_m / self._fx
        y_m = (py - self._cy) * z_m / self._fy

        return Point(x=x_m, y=y_m, z=z_m)

    def _poll(self):
        if self._nn_queue is None:
            return

        try:
            packet = self._nn_queue.tryGet()
        except RuntimeError:
            return

        if packet is None or not self._inference_active:
            return
        
        # # ─── DIAGNOSTIC PRINT ───
        # self.get_logger().info("--- NEW NN PACKET RECEIVED ---")
        # try:
        #     # Loop through all layers and print their names and shape dimensions
        #     for layer in packet.getAllLayers():
        #          self.get_logger().info(f"Layer Name: '{layer.name}' | Shape: {layer.dims}")
        # except Exception as e:
        #     self.get_logger().error(f"Failed to read layer info: {e}")
            
        # return # Temporarily stop here so it doesn't crash on the reshape line

        # 1. Parse YOLOv8 output
        layer_data = packet.getFirstTensor()
        output = np.array(layer_data, dtype=np.float32).reshape(1, 4 + NC, -1)
        detections = parse_yolov8(output, CONF_THRESH, IOU_THRESH)
        self._latest_detections = detections

        # 2. Get the latest depth map
        depth_frame = None
        if self._depth_queue is not None:
            depth_packets = self._depth_queue.tryGetAll()
            if depth_packets:
                depth_frame = depth_packets[-1].getFrame()  # 2D Numpy Array

        msg = WasteDetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "oak_rgb_camera_optical_frame"

        # 3. Build messages with custom math
        for box, score, class_id in detections:
            d = WasteDetection()
            d.label = CLASSES[class_id]
            d.confidence = float(score)
            d.bbox = [float(v) for v in box]

            point = self._get_spatial_coordinates(box, depth_frame)
            if point is not None:
                d.position = point
                d.has_3d_position = True
            else:
                d.position = Point()
                d.has_3d_position = False

            msg.detections.append(d)

        if msg.detections:
            self.publisher.publish(msg)

    def _publish_image(self):
        if self._img_queue is None:
            return

        img_packets = self._img_queue.tryGetAll()
        if not img_packets:
            return

        frame = np.ascontiguousarray(img_packets[-1].getCvFrame())

        for box, score, class_id in self._latest_detections:
            x1, y1, x2, y2 = map(int, box)
            label = f"{CLASSES[class_id]} {score:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                label,
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        img_msg = CompressedImage()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "oak_rgb_camera_optical_frame"
        img_msg.format = "jpeg"
        img_msg.data = buf.tobytes()
        self.image_pub.publish(img_msg)

    def _publish_raw_topics(self):
        stamp = self.get_clock().now().to_msg()
        frame_id = "oak_rgb_camera_optical_frame"

        # ── RGB raw ───────────────────────────────────────────────────────────
        img_packets = self._img_queue.tryGetAll() if self._img_queue else []
        if img_packets:
            frame = np.ascontiguousarray(img_packets[-1].getCvFrame())
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id
            msg.height, msg.width = frame.shape[:2]
            msg.encoding = "bgr8"
            msg.step = msg.width * 3
            msg.data = frame.tobytes()
            self.rgb_raw_pub.publish(msg)

        # ── Depth raw ─────────────────────────────────────────────────────────
        depth_packets = self._raw_depth_queue.tryGetAll() if self._raw_depth_queue else []
        if depth_packets:
            depth_frame = depth_packets[-1].getFrame()   # uint16, mm
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id
            msg.height, msg.width = depth_frame.shape[:2]
            msg.encoding = "16UC1"
            msg.step = msg.width * 2
            msg.data = depth_frame.astype(np.uint16).tobytes()
            self.depth_raw_pub.publish(msg)

        # ── CameraInfo (from EEPROM) ──────────────────────────────────────────
        ci = CameraInfo()
        ci.header.stamp = stamp
        ci.header.frame_id = frame_id
        ci.width  = INPUT_W
        ci.height = INPUT_H

        # calib = self._pipeline.getDefaultDevice().readCalibration()
        # M = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, INPUT_W, INPUT_H)
        M = self._cam_matrix

        ci.k = [M[0][0], M[0][1], M[0][2],
                M[1][0], M[1][1], M[1][2],
                M[2][0], M[2][1], M[2][2]]
        ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        ci.r = [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
        ci.p = [M[0][0], M[0][1], M[0][2], 0.0,
                M[1][0], M[1][1], M[1][2], 0.0,
                M[2][0], M[2][1], M[2][2], 0.0]
        ci.distortion_model = "plumb_bob"
        self.cam_info_pub.publish(ci)

    def destroy_node(self):
        if self._pipeline is not None:
            self._pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
