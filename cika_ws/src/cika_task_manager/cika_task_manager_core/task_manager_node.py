#!/usr/bin/env python3
"""
task_manager_node.py
────────────────────────────────────────────────────────────────────────────
Sits between perception and the rest of the system.

Subscribes to /cika/waste_detections_classified, selects the best detection,
navigates to an approach pose via Nav2 NavigateToPose, verifies the detection
is stable over N frames, then triggers the arm via PickAndDispose action.

State machine:
    IDLE → SELECTING → NAVIGATING → VERIFYING → PICKING → IDLE

After picking the robot returns to IDLE and continues searching.
Nav2 failures trigger one retry before skipping back to IDLE.
"""

import math

import rclpy
import rclpy.duration
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — required for PoseStamped transform support
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from enum import Enum, auto

from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

from cika_perception.msg import WasteDetection, WasteDetectionArray

# ── Action import — stubbed until cika_manipulator is ready ──────────────────
try:
    from cika_manipulator.action import PickAndDispose

    MANIPULATOR_AVAILABLE = True
except ImportError:
    MANIPULATOR_AVAILABLE = False


class State(Enum):
    IDLE = auto()
    SELECTING = auto()
    NAVIGATING = auto()
    VERIFYING = auto()
    PICKING = auto()


class TaskManagerNode(Node):

    def __init__(self):
        super().__init__("task_manager_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("approach_distance", 0.4)  # metres from object
        self.declare_parameter(
            "detector_conf_threshold", 0.55
        )  # min YOLO conf to consider
        self.declare_parameter("classifier_conf_threshold", 0.65)  # min classifier conf
        self.declare_parameter(
            "verify_frames_required", 5
        )  # consecutive frames to confirm
        self.declare_parameter("nav2_retry_limit", 1)  # retries before skip
        self.declare_parameter("pick_stub_duration", 3.0)  # seconds to simulate pick
        self.declare_parameter("detections_topic", "/cika/waste_detections_classified")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("global_frame", "map")

        self.approach_dist = self.get_parameter("approach_distance").value
        self.det_conf_thresh = self.get_parameter("detector_conf_threshold").value
        self.cls_conf_thresh = self.get_parameter("classifier_conf_threshold").value
        self.verify_frames_req = self.get_parameter("verify_frames_required").value
        self.nav2_retry_limit = self.get_parameter("nav2_retry_limit").value
        self.pick_stub_duration = self.get_parameter("pick_stub_duration").value
        self.base_frame = self.get_parameter("robot_base_frame").value
        self.global_frame = self.get_parameter("global_frame").value

        # ── State ─────────────────────────────────────────────────────────────
        self._state = State.IDLE
        self._target: WasteDetection | None = None
        self._nav2_retries = 0
        self._verify_count = 0
        self._nav2_goal_handle = None

        # ── TF2 ───────────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Action clients ────────────────────────────────────────────────────
        self._nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        if MANIPULATOR_AVAILABLE:
            self._arm_client = ActionClient(self, PickAndDispose, "pick_and_dispose")
            self.get_logger().info("cika_manipulator found — arm actions enabled")
        else:
            self._arm_client = None
            self.get_logger().warn(
                "cika_manipulator not found — arm pick will be STUBBED "
                "(swap stub when cika_manipulator is ready)"
            )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            WasteDetectionArray,
            self.get_parameter("detections_topic").value,
            self._detections_cb,
            10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        # State topic — useful for RViz overlays and debugging
        self._state_pub = self.create_publisher(String, "/cika/task_state", 10)

        # Status timer — publishes state at 1 Hz
        self.create_timer(1.0, self._publish_state)

        self.get_logger().info("TaskManagerNode ready | state: IDLE")

    # ── State publisher ────────────────────────────────────────────────────────
    def _publish_state(self):
        msg = String()
        msg.data = self._state.name
        self._state_pub.publish(msg)

    # ── Detection callback ─────────────────────────────────────────────────────
    def _detections_cb(self, msg: WasteDetectionArray):
        if self._state == State.IDLE or self._state == State.SELECTING:
            self._try_select(msg)
        elif self._state == State.VERIFYING:
            self._verify(msg)

    # ── IDLE → SELECTING → NAVIGATING ─────────────────────────────────────────
    def _try_select(self, msg: WasteDetectionArray):
        """Pick the best detection — highest combined confidence with valid 3D position."""
        candidates = [
            d
            for d in msg.detections
            if d.has_3d_position
            and d.confidence_detector >= self.det_conf_thresh
            and (
                d.confidence_classifier == 0.0  # classifier not yet scored
                or d.confidence_classifier >= self.cls_conf_thresh
            )
        ]

        if not candidates:
            return

        # Score = detector_conf * 0.6 + classifier_conf * 0.4
        # If classifier hasn't scored yet, weight purely on detector
        def score(d: WasteDetection) -> float:
            cls_conf = (
                d.confidence_classifier
                if d.confidence_classifier > 0.0
                else d.confidence_detector
            )
            return d.confidence_detector * 0.6 + cls_conf * 0.4

        best = max(candidates, key=score)
        self._target = best
        self._state = State.SELECTING

        self.get_logger().info(
            f"Selected target | label: {best.label} "
            f"| det_conf: {best.confidence_detector:.2f} "
            f"| cls_conf: {best.confidence_classifier:.2f} "
            f"| pos: ({best.position.x:.2f}, {best.position.y:.2f}, {best.position.z:.2f})"
        )

        self._navigate_to_target(best)

    # ── Navigation ─────────────────────────────────────────────────────────────
    def _navigate_to_target(self, det: WasteDetection):
        """Compute approach pose and send NavigateToPose goal."""
        if not self._nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 NavigateToPose action server not available — returning to IDLE")
            self._reset()
            return

        approach = self._compute_approach_pose(det.position)
        if approach is None:
            self.get_logger().error("Could not compute approach pose — returning to IDLE")
            self._reset()
            return

        # Transform approach pose from base_link → map frame for Nav2
        try:
            approach_map = self.tf_buffer.transform(
                approach,
                self.global_frame,
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except Exception as exc:
            self.get_logger().error(f"TF2 transform base_link → map failed: {exc} — returning to IDLE")
            self._reset()
            return

        goal = NavigateToPose.Goal()
        goal.pose = approach_map

        self._state = State.NAVIGATING
        self.get_logger().info(
            f"Navigating to approach pose | "
            f"x: {approach.pose.position.x:.2f} "
            f"y: {approach.pose.position.y:.2f} "
            f"| retry: {self._nav2_retries}/{self.nav2_retry_limit}"
        )

        send_future = self._nav2_client.send_goal_async(
            goal,
            feedback_callback=self._nav2_feedback_cb,
        )
        send_future.add_done_callback(self._nav2_goal_response_cb)

    def _compute_approach_pose(self, obj_pos: Point) -> PoseStamped | None:
        """
        Compute a PoseStamped approach point approach_dist metres away from
        the object, facing the object, in the map frame.

        Since obj_pos is in base_link, we compute the direction vector and
        back off by approach_dist. The result is expressed in map frame via
        the current robot pose — for now we use base_link directly and rely
        on Nav2's transform tolerance.
        """
        # Direction from robot (origin of base_link) to object
        dx = obj_pos.x
        dy = obj_pos.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.01:
            self.get_logger().warn("Object too close to robot origin — skipping")
            return None

        # Unit vector toward object
        ux = dx / dist
        uy = dy / dist

        # Approach point: stop approach_dist short of the object
        approach_x = dx - ux * self.approach_dist
        approach_y = dy - uy * self.approach_dist

        # Heading: face the object
        yaw = math.atan2(uy, ux)

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = approach_x
        pose.pose.position.y = approach_y
        pose.pose.position.z = 0.0

        # Quaternion from yaw
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def _nav2_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected goal")
            self._handle_nav2_failure()
            return
        self._nav2_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_result_cb)

    def _nav2_feedback_cb(self, feedback_msg):
        # Could log distance remaining here if needed
        pass

    def _nav2_result_cb(self, future):
        result = future.result()
        status = result.status

        # action_msgs/GoalStatus: SUCCEEDED=4, CANCELED=5, ABORTED=6
        if status == 4:
            self.get_logger().info("Nav2 reached approach pose — starting verification")
            self._state = State.VERIFYING
            self._verify_count = 0
        else:
            self.get_logger().warn(f"Nav2 failed with status {status}")
            self._handle_nav2_failure()

    def _handle_nav2_failure(self):
        if self._nav2_retries < self.nav2_retry_limit:
            self._nav2_retries += 1
            self.get_logger().info(
                f"Retrying navigation (attempt {self._nav2_retries})"
            )
            if self._target is not None:
                self._navigate_to_target(self._target)
        else:
            self.get_logger().warn("Nav2 retry limit reached — returning to IDLE")
            self._reset()

    # ── Verification ───────────────────────────────────────────────────────────
    def _verify(self, msg: WasteDetectionArray):
        """
        Confirm the target is still visible and confident over N consecutive
        frames before triggering the arm.
        """
        if self._target is None:
            self._reset()
            return

        # Check if any detection still matches our target label with sufficient confidence
        confirmed = any(
            d.label == self._target.label
            and d.confidence_detector >= self.det_conf_thresh
            and d.has_3d_position
            for d in msg.detections
        )

        if confirmed:
            self._verify_count += 1
            self.get_logger().info(
                f"Verifying target | {self._verify_count}/{self.verify_frames_req} frames"
            )
            if self._verify_count >= self.verify_frames_req:
                self.get_logger().info("Target verified — triggering arm pick")
                self._state = State.PICKING
                self._trigger_pick()
        else:
            self.get_logger().warn(
                "Target lost during verification — returning to IDLE"
            )
            self._reset()

    # ── Pick ───────────────────────────────────────────────────────────────────
    def _trigger_pick(self):
        if MANIPULATOR_AVAILABLE and self._arm_client is not None:
            self._trigger_pick_real()
        else:
            self._trigger_pick_stub()

    def _trigger_pick_real(self):
        """Send PickAndDispose goal to cika_manipulator action server."""
        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "pick_and_dispose action server not available — using stub"
            )
            self._trigger_pick_stub()
            return

        goal = PickAndDispose.Goal()
        goal.target_position = self._target.position
        goal.waste_label = self._target.label

        send_future = self._arm_client.send_goal_async(
            goal,
            feedback_callback=self._arm_feedback_cb,
        )
        send_future.add_done_callback(self._arm_goal_response_cb)

    def _arm_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Arm phase: {fb.current_phase} | progress: {fb.progress:.0%}",
            throttle_duration_sec=1.0,
        )

    def _arm_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Arm action rejected — returning to IDLE")
            self._reset()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._arm_result_cb)

    def _arm_result_cb(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f"Pick complete: {result.message} — returning to IDLE"
            )
        else:
            self.get_logger().warn(f"Pick failed: {result.message} — returning to IDLE")
        self._reset()


    def _trigger_pick_stub(self):
        self.get_logger().warn(
            f"[STUB] Arm pick triggered for '{self._target.label}' at "
            f"({self._target.position.x:.2f}, {self._target.position.y:.2f}, "
            f"{self._target.position.z:.2f}) — simulating {self.pick_stub_duration}s pick"
        )
        self._stub_timer = self.create_timer(
            self.pick_stub_duration,
            self._stub_pick_complete,
        )

    def _stub_pick_complete(self):
        self._stub_timer.cancel()  # cancel before reset
        self.get_logger().info("[STUB] Pick complete — returning to IDLE")
        self._reset()

    # ── Reset ──────────────────────────────────────────────────────────────────
    def _reset(self):
        self._state = State.IDLE
        self._target = None
        self._nav2_retries = 0
        self._verify_count = 0
        self._nav2_goal_handle = None
        self.get_logger().info("State → IDLE | searching for waste")


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
