#!/usr/bin/env python3
"""
classifier_node.py
────────────────────────────────────────────────────────────────────────────
Subscribes to WasteDetectionArray from detector_node + the live RGB image.
For each detection, crops the bounding box region and runs the binary
classifier (EfficientNet-B0 or ResNet50, trained on TrashNet) to verify
the recyclable / non_recyclable label.

Checkpoint format expected (confirmed from your .pth files):
    {
        'epoch': ...,
        'model_state_dict': OrderedDict(...),
        'optimizer_state_dict': ...,
        'val_acc': ...,
        'history': ...
    }

Publishes enriched WasteDetectionArray on /cika/waste_detections_classified.

If classifier confidence < threshold, the detector label is kept unchanged —
the classifier only overrides when it is confident.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import torch
import torch.nn as nn
import torchvision.transforms as T
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from cika_perception.msg import WasteDetection, WasteDetectionArray


# ── ImageNet normalisation — must match training preprocessing ────────────────
PREPROCESS = T.Compose([
    T.ToPILImage(),
    T.Resize((224, 224)),
    T.ToTensor(),
    T.Normalize(mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]),
])

# Binary classifier output — must match your TrashNet split
CLASS_NAMES = {0: "recyclable", 1: "non_recyclable"}

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=5,
)


class ClassifierNode(Node):

    def __init__(self):
        super().__init__("classifier_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("model_path", "")
        self.declare_parameter("model_type", "efficientnet")  # or "resnet50"
        self.declare_parameter("confidence_threshold", 0.70)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("detections_input_topic",  "/cika/waste_detections")
        self.declare_parameter("rgb_topic",               "/oak/rgb/image_raw")
        self.declare_parameter("detections_output_topic", "/cika/waste_detections_classified")

        model_path       = self.get_parameter("model_path").value
        self.model_type  = self.get_parameter("model_type").value
        self.conf_thresh = self.get_parameter("confidence_threshold").value
        device_str       = self.get_parameter("device").value
        self.device      = torch.device("cuda" if device_str == "cuda" else "cpu")

        # ── Model ─────────────────────────────────────────────────────────────
        self.model = self._load_model(model_path)

        # ── State ─────────────────────────────────────────────────────────────
        self.bridge       = CvBridge()
        self._latest_rgb: np.ndarray | None = None

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Image,
            self.get_parameter("rgb_topic").value,
            self._rgb_cb,
            SENSOR_QOS,
        )
        self.create_subscription(
            WasteDetectionArray,
            self.get_parameter("detections_input_topic").value,
            self._detections_cb,
            10,
        )

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub = self.create_publisher(
            WasteDetectionArray,
            self.get_parameter("detections_output_topic").value,
            10,
        )

        self.get_logger().info(
            f"ClassifierNode ready | model: {model_path} | type: {self.model_type} "
            f"| conf_thresh: {self.conf_thresh} | device: {self.device}"
        )

    # ── Model loading ──────────────────────────────────────────────────────────
    def _load_model(self, model_path: str):
        if not model_path:
            self.get_logger().error("model_path is empty — check perception_sim.yaml")
            return None
        try:
            if self.model_type == "efficientnet":
                from torchvision.models import efficientnet_b0
                model = efficientnet_b0(weights=None)
                # Replace head to match your training (2 output classes)
                model.classifier[1] = nn.Linear(
                    model.classifier[1].in_features, 2
                )
            elif self.model_type == "resnet50":
                from torchvision.models import resnet50
                model = resnet50(weights=None)
                model.fc = nn.Linear(model.fc.in_features, 2)
            else:
                self.get_logger().error(f"Unknown model_type: {self.model_type}")
                return None

            # Load checkpoint — your .pth files use 'model_state_dict' key
            checkpoint = torch.load(model_path, map_location=self.device)
            model.load_state_dict(checkpoint["model_state_dict"])
            model.to(self.device)
            model.eval()

            # Log checkpoint metadata for confirmation
            self.get_logger().info(
                f"Loaded {self.model_type} from {model_path} "
                f"| saved at epoch {checkpoint.get('epoch', '?')} "
                f"| val_acc {checkpoint.get('val_acc', '?')}"
            )
            return model

        except Exception as exc:
            self.get_logger().error(f"Classifier load failed: {exc}")
            return None

    # ── RGB cache ──────────────────────────────────────────────────────────────
    def _rgb_cb(self, msg: Image):
        try:
            # Keep as RGB — PREPROCESS expects RGB input (ToPILImage convention)
            self._latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as exc:
            self.get_logger().warn(f"RGB conversion failed: {exc}")

    # ── Detection callback ─────────────────────────────────────────────────────
    def _detections_cb(self, msg: WasteDetectionArray):
        out = WasteDetectionArray()
        out.header = msg.header

        for det in msg.detections:
            out.detections.append(self._classify(det))

        self.pub.publish(out)

    # ── Classify a single detection ────────────────────────────────────────────
    def _classify(self, det: WasteDetection) -> WasteDetection:
        """
        Crop the detection bbox from the latest RGB frame and run the
        classifier. Overrides det.label only if confidence >= threshold.
        Always fills det.confidence_classifier.
        """
        if self.model is None or self._latest_rgb is None:
            return det

        h, w = self._latest_rgb.shape[:2]
        x1 = max(0, int(det.bbox[0]))
        y1 = max(0, int(det.bbox[1]))
        x2 = min(w, int(det.bbox[2]))
        y2 = min(h, int(det.bbox[3]))

        if x2 <= x1 or y2 <= y1:
            self.get_logger().warn("Degenerate bounding box — skipping classification")
            return det

        crop = self._latest_rgb[y1:y2, x1:x2]   # RGB numpy array

        try:
            tensor = PREPROCESS(crop).unsqueeze(0).to(self.device)  # (1, 3, 224, 224)
            with torch.no_grad():
                logits = self.model(tensor)                          # (1, 2)
                probs  = torch.softmax(logits, dim=1)[0]             # (2,)
                cls_id = int(torch.argmax(probs).item())
                confidence = float(probs[cls_id].item())
        except Exception as exc:
            self.get_logger().warn(f"Classifier inference failed: {exc}")
            return det

        det.confidence_classifier = confidence

        # Override detector label only if classifier is confident enough
        if confidence >= self.conf_thresh:
            det.label = CLASS_NAMES.get(cls_id, det.label)

        return det


def main(args=None):
    rclpy.init(args=args)
    node = ClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()