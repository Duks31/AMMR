#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
import depthai as dai
from sensor_msgs.msg import Image

from cika_perception.msg import WasteDetectionArray, WasteDetection

BLOB_PATH   = "/cika_ws/src/cika_perception/models/300_epoch_best.blob"
INPUT_W     = 640
INPUT_H     = 640
CONF_THRESH = 0.5
IOU_THRESH  = 0.45
CLASSES     = ["plastic", "paper", "metal"]
NC          = len(CLASSES)                          # fix 3: module-level constant


def xywh2xyxy(cx, cy, w, h):
    return cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2


def nms(boxes, scores, iou_thresh):
    if len(boxes) == 0:
        return []
    boxes  = np.array(boxes,  dtype=np.float32)
    scores = np.array(scores, dtype=np.float32)
    x1, y1, x2, y2 = boxes[:,0], boxes[:,1], boxes[:,2], boxes[:,3]
    areas  = (x2 - x1) * (y2 - y1)
    order  = scores.argsort()[::-1]
    keep   = []
    while order.size > 0:
        i   = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w_  = np.maximum(0.0, xx2 - xx1)           # fix 4: renamed to w_
        h_  = np.maximum(0.0, yy2 - yy1)           # fix 4: renamed to h_
        iou = (w_ * h_) / (areas[i] + areas[order[1:]] - w_ * h_ + 1e-6)
        order = order[np.where(iou <= iou_thresh)[0] + 1]
    return keep


def parse_yolov8(output: np.ndarray, conf_thresh: float, iou_thresh: float):
    pred = output[0].T  # [8400, 4+NC]
    boxes, scores, class_ids = [], [], []
    for row in pred:
        cx, cy, w, h = row[0], row[1], row[2], row[3]
        class_scores = row[4:]
        class_id     = int(np.argmax(class_scores))
        confidence   = float(class_scores[class_id])
        if confidence < conf_thresh:
            continue
        x1, y1, x2, y2 = xywh2xyxy(cx, cy, w, h)
        boxes.append([x1, y1, x2, y2])
        scores.append(confidence)
        class_ids.append(class_id)
    keep = nms(boxes, scores, iou_thresh)
    return [(boxes[i], scores[i], class_ids[i]) for i in keep]


class InferenceNode(Node):
    def __init__(self):
        super().__init__('inference_node')
        self.publisher  = self.create_publisher(
            WasteDetectionArray,
            '/cika/perception/waste_detections',
            10)
        self._device    = None                      # fix 5: init to None
        self._nn_queue  = None
        self._img_queue = None
        self._start_pipeline()
        self.create_timer(0.033, self._poll)
        self.get_logger().info("Inference Node Ready. Listening to OAK-D Lite...")
        self.image_pub = self.create_publisher(Image, '/cika/perception/image_raw', 10)

    def _start_pipeline(self):
        pipeline = dai.Pipeline()

        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(INPUT_W, INPUT_H)
        cam.setVideoSize(INPUT_W, INPUT_H)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(15)

        # NN — linked to preview
        nn = pipeline.create(dai.node.NeuralNetwork)
        nn.setBlobPath(BLOB_PATH)
        nn.setNumInferenceThreads(2)
        cam.preview.link(nn.input)

        nn_out = pipeline.create(dai.node.XLinkOut)
        nn_out.setStreamName("nn")
        nn.out.link(nn_out.input)

        # Image — linked to video (separate output from preview)
        img_out = pipeline.create(dai.node.XLinkOut)
        img_out.setStreamName("rgb")
        cam.video.link(img_out.input)   # video, NOT preview
    
        # Single device creation — after all nodes are defined
        self._device    = dai.Device(pipeline)
        self._nn_queue  = self._device.getOutputQueue("nn",  maxSize=4, blocking=False)
        self._img_queue = self._device.getOutputQueue("rgb", maxSize=4, blocking=False)
        
    def _poll(self):
        if self._nn_queue is None:
            return
        packet = self._nn_queue.tryGet()
        if packet is None:
            return

        try:
            layers = packet.getAllLayers()
            if not layers:
                return
            layer_data = packet.getLayerFp16(layers[0].name)
            output = np.array(layer_data, dtype=np.float32).reshape(1, 4 + NC, -1)
        except Exception as e:
            self.get_logger().warn(
                f"Layer read failed: {e}", throttle_duration_sec=5.0)
            return

        detections = parse_yolov8(output, CONF_THRESH, IOU_THRESH)

        # Publish camera frame
        if self._img_queue is not None:
            img_packet = self._img_queue.tryGet()
            if img_packet is not None:
                frame = img_packet.getCvFrame()
                img_msg = Image()
                img_msg.header.stamp    = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "oak_rgb_camera_optical_frame"
                img_msg.height          = frame.shape[0]
                img_msg.width           = frame.shape[1]
                img_msg.encoding        = "bgr8"
                img_msg.step            = frame.shape[1] * 3
                img_msg.data            = frame.tobytes()
                self.image_pub.publish(img_msg)

        msg = WasteDetectionArray()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "oak_rgb_camera_optical_frame"

        for (box, score, class_id) in detections:
            d                 = WasteDetection()
            d.label           = CLASSES[class_id]
            d.confidence      = float(score)
            d.bbox            = [float(v) for v in box]
            d.has_3d_position = False
            msg.detections.append(d)

        if msg.detections:
            self.publisher.publish(msg)

    def destroy_node(self):
        if self._device is not None:                # fix 5: guard against None
            self._device.close()
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

if __name__ == '__main__':
    main()