#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from cika_perception.msg import WasteDetectionArray, WasteDetection

class InferenceNode(Node):
    def __init__(self):
        super().__init__('inference_node')

        # ── Configuration ──────────────────────────────────────────
        self.class_map = {
            0: "plastic",
            1: "paper",
            2: "metal"
        }
            
        self.confidence_threshold = 0.60  

        # Listen to the neural network output from depthai-ros
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/oak/nn/detections', 
            self.detection_callback,
            10)

        # Publish to your custom topic
        self.publisher = self.create_publisher(
            WasteDetectionArray,
            '/cika/perception/waste_detections',
            10)

        self.get_logger().info("Inference Node Ready. Listening to OAK-D Lite...")

    def detection_callback(self, msg):
        waste_array_msg = WasteDetectionArray()
        waste_array_msg.header = msg.header

        for detection in msg.detections:
            if not detection.results:
                continue

            best_result = detection.results[0]
            
            try:
                class_id = int(best_result.hypothesis.class_id)
            except ValueError:
                continue

            score = best_result.hypothesis.score

            if score < self.confidence_threshold:
                continue

            # Populate the clean, modernized message
            waste_msg = WasteDetection()
            waste_msg.label = self.class_map.get(class_id, "unknown")
            waste_msg.confidence = float(score)

            # Convert to [xmin, ymin, xmax, ymax]
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            w = detection.bbox.size_x
            h = detection.bbox.size_y

            waste_msg.bbox = [
                float(cx - w / 2.0),
                float(cy - h / 2.0),
                float(cx + w / 2.0),
                float(cy + h / 2.0)
            ]

            waste_msg.has_3d_position = False

            waste_array_msg.detections.append(waste_msg)

        if waste_array_msg.detections:
            self.publisher.publish(waste_array_msg)

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