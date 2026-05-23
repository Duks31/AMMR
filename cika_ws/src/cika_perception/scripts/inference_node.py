#!/usr/bin/env python3

import os
import numpy as np
import rclpy
import cv2
from rclpy.node import Node
import depthai as dai
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import SetBool
from ament_index_python.packages import get_package_share_directory

from cika_perception.msg import WasteDetectionArray, WasteDetection
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

package_share_directory = get_package_share_directory("cika_perception")

BLOB_PATH = os.path.join(package_share_directory, "models", "taco_2class_300epoch.blob")
INPUT_W = 640
INPUT_H = 640
CONF_THRESH = 0.5
IOU_THRESH = 0.45
CLASSES = ["plastic", "paper"]
NC = len(CLASSES)

# Depth thresholds in mm
DEPTH_MIN_MM = 100
DEPTH_MAX_MM = 8000

# ── Hardware profile ──────────────────────────────────────────────────────────
# Set to True when running on Raspberry Pi 4B, False for laptop testing
ON_PI = True

# ── Camera settings (auto-configured by profile) ──────────────────────────────
CAM_FPS = 30 if not ON_PI else 10
MONO_RES = dai.MonoCameraProperties.SensorResolution.THE_400_P
LRC_ENABLED = True
USB_SPEED = dai.UsbSpeed.HIGH

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1
)


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


def bbox_to_normalised_roi(box, frame_w=INPUT_W, frame_h=INPUT_H):
    """Convert pixel bbox [x1,y1,x2,y2] to a normalised dai.Rect clamped to [0,1]."""
    x1, y1, x2, y2 = box
    return dai.Rect(
        dai.Point2f(max(0.001, x1 / frame_w), max(0.001, y1 / frame_h)),
        dai.Point2f(min(0.999, x2 / frame_w), min(0.999, y2 / frame_h)),
    )


class InferenceNode(Node):
    def __init__(self):
        super().__init__("inference_node")
        self.publisher = self.create_publisher(
            WasteDetectionArray, "/cika/perception/waste_detections", 10
        )
        self._device = None
        self._nn_queue = None
        self._img_queue = None
        self._spatial_calc_cfg_queue = None  # host → device: ROI configs
        self._spatial_calc_out_queue = None  # device → host: 3D locations
        self._inference_active = False
        self._start_pipeline()
        self.create_timer(0.1, self._poll)
        self.create_timer(0.1, self._publish_image)
        self.create_service(
            SetBool, "/cika/perception/set_inference_active", self._enable_cb
        )
        self.get_logger().info("Inference Node Ready. Listening to OAK-D Lite...")
        self.image_pub = self.create_publisher(
            CompressedImage, "/cika/perception/image_raw/compressed", qos
        )
        self.depth_pub = self.create_publisher(
            CompressedImage, "/cika/perception/depth/compressed", qos
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
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(INPUT_W, INPUT_H)
        # cam.setVideoSize(INPUT_W, INPUT_H)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(CAM_FPS)

        # ── Mono cameras (left/right for stereo) ──────────────────────────────
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_left.setResolution(MONO_RES)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)

        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_right.setResolution(MONO_RES)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        mono_left.setFps(5)
        mono_right.setFps(5)

        # ── StereoDepth ───────────────────────────────────────────────────────
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Align depth map to the RGB camera frame so pixel coords match
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(INPUT_W, INPUT_H)
        stereo.setLeftRightCheck(LRC_ENABLED)
        stereo.setSubpixel(False)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # ── SpatialLocationCalculator ─────────────────────────────────────────
        # Accepts ROI configs from host, returns x/y/z in mm for each ROI
        spatial_calc = pipeline.create(dai.node.SpatialLocationCalculator)
        spatial_calc.inputConfig.setWaitForMessage(False)
        stereo.depth.link(spatial_calc.inputDepth)

        # ── NeuralNetwork ─────────────────────────────────────────────────────
        nn = pipeline.create(dai.node.NeuralNetwork)
        nn.setBlobPath(BLOB_PATH)
        nn.setNumInferenceThreads(2)
        cam.preview.link(nn.input)

        # ── XLinkOut: NN results ──────────────────────────────────────────────
        nn_out = pipeline.create(dai.node.XLinkOut)
        nn_out.setStreamName("nn")
        nn.out.link(nn_out.input)
        
        # ── XLinkOut: RGB video ───────────────────────────────────────────────
        img_out = pipeline.create(dai.node.XLinkOut)
        img_out.setStreamName("rgb")
        cam.preview.link(img_out.input)

        img_out = pipeline.create(dai.node.XLinkOut)
        img_out.setStreamName("rgb")
        video_enc.bitstream.link(img_out.input)
        # img_out = pipeline.create(dai.node.XLinkOut)
        # img_out.setStreamName("rgb")
        # cam.preview.link(img_out.input)
        # nn.passthrough.link(img_out.input)  # Get RGB frames synchronized with NN results

        # --- ADD THESE 3 LINES ---
        # depth_out = pipeline.create(dai.node.XLinkOut)
        # depth_out.setStreamName("depth")
        # stereo.depth.link(depth_out.input)

        # ── XLinkOut: spatial locations ───────────────────────────────────────
        spatial_out = pipeline.create(dai.node.XLinkOut)
        spatial_out.setStreamName("spatial_data")
        spatial_calc.out.link(spatial_out.input)

        # ── XLinkIn: spatial ROI configs (host → device) ──────────────────────
        spatial_in = pipeline.create(dai.node.XLinkIn)
        spatial_in.setStreamName("spatial_cfg")
        spatial_in.out.link(spatial_calc.inputConfig)

        # ── Start device ──────────────────────────────────────────────────────
        self._device = dai.Device(pipeline, USB_SPEED)
        self._nn_queue = self._device.getOutputQueue("nn", maxSize=1, blocking=False)
        self._img_queue = self._device.getOutputQueue("rgb", maxSize=1, blocking=False)
        # self._depth_queue = self._device.getOutputQueue(
        #     "depth", maxSize=1, blocking=False
        # )
        self._spatial_calc_out_queue = self._device.getOutputQueue(
            "spatial_data", maxSize=8, blocking=False
        )
        self._spatial_calc_cfg_queue = self._device.getInputQueue(
            "spatial_cfg", maxSize=8, blocking=False
        )

    def _query_3d_positions(self, detections):
        """
        Send all detection bboxes as ROIs to SpatialLocationCalculator in one
        config message, read back one result message, return list of Point or None.
        Returns a list the same length as detections.
        """
        if not detections:
            return []

        cfg = dai.SpatialLocationCalculatorConfig()
        roi_cfgs = []
        for box, _, _ in detections:
            roi = bbox_to_normalised_roi(box)
            roi_data = dai.SpatialLocationCalculatorConfigData()
            roi_data.roi = roi
            roi_data.depthThresholds.lowerThreshold = DEPTH_MIN_MM
            roi_data.depthThresholds.upperThreshold = DEPTH_MAX_MM
            roi_cfgs.append(roi_data)
        cfg.setROIs(roi_cfgs)
        self._spatial_calc_cfg_queue.send(cfg)

        # Non-blocking read — if result not ready this frame, skip 3D for now
        result = self._spatial_calc_out_queue.tryGet()
        if result is None:
            return [None] * len(detections)

        points = []
        for loc in result.getSpatialLocations():
            sp = loc.spatialCoordinates
            # SpatialLocationCalculator returns mm → convert to metres for ROS
            points.append(Point(x=sp.x / 1000.0, y=sp.y / 1000.0, z=sp.z / 1000.0))

        # Pad with None if result count doesn't match (shouldn't happen, but safe)
        while len(points) < len(detections):
            points.append(None)
        return points

    def _poll(self):
        if self._nn_queue is None:
            return
        
        self._spatial_calc_out_queue.tryGetAll()

        try:
            packet = self._nn_queue.tryGet()
        except RuntimeError as e:
            self.get_logger().error(f"XLink error: {e}", throttle_duration_sec=5.0)
            return
        if packet is None:
            return

        # ros2 service call /cika/perception/set_inference_active std_srvs/srv/SetBool "{data: true}"
        # ros2 service call /cika/perception/set_inference_active std_srvs/srv/SetBool "{data: false}"
        if not self._inference_active:
            return

        try:
            layers = packet.getAllLayers()
            if not layers:
                return
            layer_data = packet.getLayerFp16(layers[0].name)
            output = np.array(layer_data, dtype=np.float32).reshape(1, 4 + NC, -1)
        except Exception as e:
            self.get_logger().warn(f"Layer read failed: {e}", throttle_duration_sec=5.0)
            return

        detections = parse_yolov8(output, CONF_THRESH, IOU_THRESH)
        self._latest_detections = (
            detections  # Store for debugging/visualization in foxglove
        )

        # Get 3D positions for all detections in one round-trip
        positions = self._query_3d_positions(detections)

        msg = WasteDetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "oak_rgb_camera_optical_frame"

        for (box, score, class_id), point in zip(detections, positions):
            d = WasteDetection()
            d.label = CLASSES[class_id]
            d.confidence = float(score)
            d.bbox = [float(v) for v in box]
            if point is not None and point.z > 0.0:
                d.position = point
                d.has_3d_position = True
            else:
                d.position = Point()
                d.has_3d_position = False
            msg.detections.append(d)

        if msg.detections:
            self.publisher.publish(msg)

    def _publish_image(self):
        if self._img_queue is not None:
            img_packets = self._img_queue.tryGetAll() # Grab all waiting frames
            if img_packets:
                img_packet = img_packets[-1]          # Keep only the newest one

                # Decode it so OpenCV can draw the bounding boxes on it
                frame = np.ascontiguousarray(img_packet.getCvFrame())
                # --------------------------------------------

                # Draw detections on the image for visualization in Foxglove Studio
                if hasattr(self, "_latest_detections"):
                    for box, score, class_id in self._latest_detections:
                        x1, y1, x2, y2 = map(int, box)
                        label = f"{CLASSES[class_id]} {score:.2f}"
                        # Draw green box and label
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

        # ─── DEPTH PUBLISHING (HEATMAP) ───────────────────────────────────────
        # if getattr(self, "_depth_queue", None) is not None:
        #     # tryGetAll() grabs every frame currently waiting in the "traffic jam"
        #     depth_packets = self._depth_queue.tryGetAll()
            
        #     if depth_packets:
        #         # [-1] means "take the very last (newest) item" and discard the rest
        #         depth_packet = depth_packets[-1]
                
        #         depth_frame = depth_packet.getFrame()  # Raw 16-bit mm data

        #         # Faster normalization using OpenCV
        #         depth_norm = cv2.normalize(
        #             depth_frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
        #         )
        #         depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

        #         _, depth_buf = cv2.imencode(
        #             ".jpg", depth_color, [cv2.IMWRITE_JPEG_QUALITY, 50]
        #         )
        #         depth_msg = CompressedImage()
        #         depth_msg.header.stamp = self.get_clock().now().to_msg()
        #         depth_msg.header.frame_id = "oak_rgb_camera_optical_frame"
        #         depth_msg.format = "jpeg"
        #         depth_msg.data = depth_buf.tobytes()
        #         self.depth_pub.publish(depth_msg)

    def destroy_node(self):
        try:
            if self._device is not None:
                self._device.close()
        except Exception:
            pass
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
