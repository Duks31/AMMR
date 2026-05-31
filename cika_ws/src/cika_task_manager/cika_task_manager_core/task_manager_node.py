#!/usr/bin/env python3
"""
task_manager_node.py
────────────────────────────────────────────────────────────────────────────
Orchestrates Cika's waste-collection loop.

Subscribes to /cika/perception/waste_detections (WasteDetectionArray),
selects the best detection, navigates to an approach pose via Nav2, verifies
the detection over N consecutive frames, then drives the arm through the full
pick-and-dispose sequence via RobotarmTask.

State machine:
    IDLE → SELECTING → NAVIGATING → VERIFYING → PICKING → IDLE

Arm sequence (RobotarmTask.action, task_server):
    Step 1 — task 4        : rest (standby position)
    Step 2 — task 3 + xyz  : dynamic pick at detected 3D position
    Step 3 — task 0        : home + open gripper
    Step 4 — task 1 or 2   : drop_1 (plastic) or drop_2 (paper)
    Step 5 — task 4        : rest

Fixes over the original scaffolded version:
  1. Field names:         confidence_detector/classifier → single confidence field
  2. Frame transform:     camera optical frame → base_link via TF2 before geometry
  3. Arm interface:       PickAndDispose replaced with RobotarmTask (task_server)
  4. Arm sequence:        full 5-step sequence matching task_server task logic
  5. Label→drop mapping:  plastic→task 1, paper→task 2
  6. Stub timer safety:   _stub_timer guarded in _reset()
  7. Nav2 cancel:         active goal cancelled on reset
  8. Detection topic:     /cika/perception/waste_detections
  9. base_footprint:      approach pose stamped in nav_base_frame (base_footprint)
                          while camera TF target remains base_link
"""

import math

import rclpy
import rclpy.duration
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support
from rclpy.node import Node
from rclpy.action import ActionClient
from enum import Enum, auto

from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

from cika_perception.msg import WasteDetection, WasteDetectionArray
from cika_manipulator.action import RobotarmTask

# Task number constants — mirrors task_server.cpp execute() logic
TASK_REST  = 4   # arm to rest position, gripper closed
TASK_PICK  = 3   # move to xyz pose, gripper closed (actual pick)
TASK_HOME  = 0   # arm to home, gripper open
TASK_DROP  = {   # label → drop bin task number
    "plastic": 1,
    "paper":   2,
}
TASK_DROP_DEFAULT = 4  # unknown label → rest (safe fallback)


class State(Enum):
    IDLE       = auto()
    SELECTING  = auto()
    NAVIGATING = auto()
    VERIFYING  = auto()
    PICKING    = auto()


class TaskManagerNode(Node):

    def __init__(self):
        super().__init__("task_manager_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("approach_distance",      0.2)   # m — stop short of object
        self.declare_parameter("conf_threshold",         0.55)  # minimum YOLO confidence
        self.declare_parameter("verify_frames_required", 5)     # consecutive frames to confirm
        self.declare_parameter("nav2_retry_limit",       1)
        self.declare_parameter("pick_stub_duration",     3.0)   # s — simulated pick time
        self.declare_parameter("detections_topic",       "/cika/perception/waste_detections")
        # camera_target_frame: frame the camera optical frame is anchored to in the URDF.
        # TF lookup: oak_rgb_camera_optical_frame → base_link (not base_footprint).
        self.declare_parameter("camera_target_frame",    "base_link")
        self.declare_parameter("camera_frame",           "oak_rgb_camera_optical_frame")
        # nav_base_frame: must match Nav2 robot_base_frame — base_footprint for Cika.
        self.declare_parameter("nav_base_frame",         "base_footprint")
        self.declare_parameter("global_frame",           "map")

        self.approach_dist       = self.get_parameter("approach_distance").value
        self.conf_thresh         = self.get_parameter("conf_threshold").value
        self.verify_frames_req   = self.get_parameter("verify_frames_required").value
        self.nav2_retry_limit    = self.get_parameter("nav2_retry_limit").value
        self.pick_stub_duration  = self.get_parameter("pick_stub_duration").value
        self.camera_target_frame = self.get_parameter("camera_target_frame").value
        self.camera_frame        = self.get_parameter("camera_frame").value
        self.nav_base_frame      = self.get_parameter("nav_base_frame").value
        self.global_frame        = self.get_parameter("global_frame").value

        # ── State ─────────────────────────────────────────────────────────────
        self._state             : State                  = State.IDLE
        self._target            : WasteDetection | None  = None
        self._nav2_retries      : int                    = 0
        self._verify_count      : int                    = 0
        self._nav2_goal_handle                           = None
        self._stub_timer                                 = None

        # ── TF2 ───────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Action clients ────────────────────────────────────────────────────
        self._nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._arm_client  = ActionClient(self, RobotarmTask,   "task_server")

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            WasteDetectionArray,
            self.get_parameter("detections_topic").value,
            self._detections_cb,
            10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self._state_pub = self.create_publisher(String, "/cika/task_state", 10)
        self.create_timer(1.0, self._publish_state)

        self.get_logger().info("TaskManagerNode ready | state: IDLE")

    # ── State publisher ────────────────────────────────────────────────────────
    def _publish_state(self):
        msg = String()
        msg.data = self._state.name
        self._state_pub.publish(msg)

    # ── Detection callback ─────────────────────────────────────────────────────
    def _detections_cb(self, msg: WasteDetectionArray):
        if self._state in (State.IDLE, State.SELECTING):
            self._try_select(msg)
        elif self._state == State.VERIFYING:
            self._verify(msg)

    # ── SELECTING ─────────────────────────────────────────────────────────────
    def _try_select(self, msg: WasteDetectionArray):
        """Pick the highest-confidence detection that has a valid 3D position."""
        candidates = [
            d for d in msg.detections
            if d.has_3d_position and d.confidence >= self.conf_thresh
        ]
        if not candidates:
            return

        best = max(candidates, key=lambda d: d.confidence)
        self._target = best
        self._state  = State.SELECTING

        self.get_logger().info(
            f"Target selected | label: {best.label} "
            f"| conf: {best.confidence:.2f} "
            f"| camera-frame pos: "
            f"({best.position.x:.2f}, {best.position.y:.2f}, {best.position.z:.2f})"
        )
        self._navigate_to_target(best)

    # ── NAVIGATING ────────────────────────────────────────────────────────────
    def _navigate_to_target(self, det: WasteDetection):
        """
        Transform the object from the camera optical frame into base_link,
        compute a stop-short approach pose, transform to map, send Nav2 goal.
        """
        if not self._nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 server not available — returning to IDLE")
            self._reset()
            return

        # Step 1 — camera optical frame → base_link
        obj_in_camera                     = PoseStamped()
        obj_in_camera.header.frame_id     = self.camera_frame
        obj_in_camera.header.stamp        = self.get_clock().now().to_msg()
        obj_in_camera.pose.position       = det.position
        obj_in_camera.pose.orientation.w  = 1.0

        try:
            obj_in_base = self.tf_buffer.transform(
                obj_in_camera,
                self.camera_target_frame,
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except Exception as exc:
            self.get_logger().error(
                f"TF2 {self.camera_frame} → {self.camera_target_frame} failed: {exc}"
                " — returning to IDLE"
            )
            self._reset()
            return

        # Step 2 — approach pose in nav_base_frame (base_footprint)
        approach_in_base = self._compute_approach_pose(obj_in_base.pose.position)
        if approach_in_base is None:
            self.get_logger().error("Could not compute approach pose — returning to IDLE")
            self._reset()
            return

        # Step 3 — base_footprint → map for Nav2
        try:
            approach_in_map = self.tf_buffer.transform(
                approach_in_base,
                self.global_frame,
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except Exception as exc:
            self.get_logger().error(
                f"TF2 {self.nav_base_frame} → {self.global_frame} failed: {exc}"
                " — returning to IDLE"
            )
            self._reset()
            return

        goal      = NavigateToPose.Goal()
        goal.pose = approach_in_map

        self._state = State.NAVIGATING
        self.get_logger().info(
            f"Navigating | map-frame approach: "
            f"({approach_in_map.pose.position.x:.2f}, "
            f"{approach_in_map.pose.position.y:.2f}) "
            f"| retry {self._nav2_retries}/{self.nav2_retry_limit}"
        )

        send_future = self._nav2_client.send_goal_async(
            goal, feedback_callback=self._nav2_feedback_cb
        )
        send_future.add_done_callback(self._nav2_goal_response_cb)

    def _compute_approach_pose(self, obj_pos: Point) -> PoseStamped | None:
        """
        Given the object position in camera_target_frame (base_link), return a
        PoseStamped stamped in nav_base_frame (base_footprint) that is
        approach_dist metres short of the object, facing it.
        """
        dx   = obj_pos.x
        dy   = obj_pos.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.01:
            self.get_logger().warn("Object too close to robot origin — skipping")
            return None

        ux = dx / dist
        uy = dy / dist

        pose                       = PoseStamped()
        pose.header.frame_id       = self.nav_base_frame
        pose.header.stamp          = self.get_clock().now().to_msg()
        pose.pose.position.x       = dx - ux * self.approach_dist
        pose.pose.position.y       = dy - uy * self.approach_dist
        pose.pose.position.z       = 0.0
        yaw                        = math.atan2(uy, ux)
        pose.pose.orientation.z    = math.sin(yaw / 2.0)
        pose.pose.orientation.w    = math.cos(yaw / 2.0)
        return pose

    def _nav2_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected goal")
            self._handle_nav2_failure()
            return
        self._nav2_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._nav2_result_cb)

    def _nav2_feedback_cb(self, feedback_msg):
        pass

    def _nav2_result_cb(self, future):
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info("Nav2 reached approach pose — starting verification")
            self._state        = State.VERIFYING
            self._verify_count = 0
        else:
            self.get_logger().warn(f"Nav2 failed with status {result.status}")
            self._handle_nav2_failure()

    def _handle_nav2_failure(self):
        if self._nav2_retries < self.nav2_retry_limit:
            self._nav2_retries += 1
            self.get_logger().info(f"Retrying navigation (attempt {self._nav2_retries})")
            if self._target is not None:
                self._navigate_to_target(self._target)
        else:
            self.get_logger().warn("Nav2 retry limit reached — returning to IDLE")
            self._reset()

    # ── VERIFYING ─────────────────────────────────────────────────────────────
    def _verify(self, msg: WasteDetectionArray):
        """Confirm the target is still visible over N consecutive frames."""
        if self._target is None:
            self._reset()
            return

        confirmed = any(
            d.label == self._target.label
            and d.confidence >= self.conf_thresh
            and d.has_3d_position
            for d in msg.detections
        )

        if confirmed:
            self._verify_count += 1
            self.get_logger().info(
                f"Verifying | {self._verify_count}/{self.verify_frames_req} frames"
            )
            if self._verify_count >= self.verify_frames_req:
                self.get_logger().info("Target verified — starting arm sequence")
                self._state = State.PICKING
                self._trigger_pick()
        else:
            self.get_logger().warn("Target lost during verification — returning to IDLE")
            self._reset()

    # ── PICKING ───────────────────────────────────────────────────────────────
    def _trigger_pick(self):
        """Transform the target position and kick off the arm sequence."""
        try:
            obj_in_camera                    = PoseStamped()
            obj_in_camera.header.frame_id    = self.camera_frame
            obj_in_camera.header.stamp       = self.get_clock().now().to_msg()
            obj_in_camera.pose.position      = self._target.position
            obj_in_camera.pose.orientation.w = 1.0

            obj_in_base = self.tf_buffer.transform(
                obj_in_camera,
                self.camera_target_frame,
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except Exception as exc:
            self.get_logger().error(
                f"TF2 for arm goal failed: {exc} — using stub"
            )
            self._trigger_pick_stub()
            return

        p = obj_in_base.pose.position
        drop_task = TASK_DROP.get(self._target.label, TASK_DROP_DEFAULT)

        self.get_logger().info(
            f"Arm sequence | label: {self._target.label} "
            f"| drop task: {drop_task} "
            f"| base_link pos: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
        )

        # Full 5-step sequence matching task_server.cpp task logic:
        #   4 (rest) → 3 xyz (pick) → 0 (home) → 1|2 (drop) → 4 (rest)
        sequence = [
            (TASK_REST, 0.0,  0.0,  0.0),   # Step 1: rest / standby
            (TASK_PICK, p.x,  p.y,  p.z),   # Step 2: pick at detected position
            (TASK_HOME, 0.0,  0.0,  0.0),   # Step 3: home + open gripper
            (drop_task, 0.0,  0.0,  0.0),   # Step 4: drop into correct bin
            (TASK_REST, 0.0,  0.0,  0.0),   # Step 5: rest
        ]

        self._arm_sequence      = sequence
        self._arm_step_index    = 0
        self._send_arm_step()

    def _send_arm_step(self):
        """Send the current step in the arm sequence."""
        if self._arm_step_index >= len(self._arm_sequence):
            self.get_logger().info("Arm sequence complete — returning to IDLE")
            self._reset()
            return

        task_num, x, y, z = self._arm_sequence[self._arm_step_index]
        step_label = self._arm_step_index + 1
        total      = len(self._arm_sequence)

        self.get_logger().info(
            f"Arm step {step_label}/{total} | task: {task_num} "
            f"| xyz: ({x:.3f}, {y:.3f}, {z:.3f})"
        )

        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("task_server not available — using stub")
            self._trigger_pick_stub()
            return

        goal             = RobotarmTask.Goal()
        goal.task_number = task_num
        goal.x           = x
        goal.y           = y
        goal.z           = z

        send_future = self._arm_client.send_goal_async(goal)
        send_future.add_done_callback(self._arm_goal_response_cb)

    def _arm_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f"Arm step {self._arm_step_index + 1} rejected — aborting sequence"
            )
            self._reset()
            return
        goal_handle.get_result_async().add_done_callback(self._arm_result_cb)

    def _arm_result_cb(self, future):
        result = future.result().result
        step   = self._arm_step_index + 1
        total  = len(self._arm_sequence)

        if result.success:
            self.get_logger().info(f"Arm step {step}/{total} succeeded")
            self._arm_step_index += 1
            self._send_arm_step()          # advance to next step
        else:
            self.get_logger().warn(
                f"Arm step {step}/{total} failed — aborting sequence"
            )
            self._reset()

    def _trigger_pick_stub(self):
        self.get_logger().warn(
            f"[STUB] Pick triggered for '{self._target.label}' — "
            f"simulating {self.pick_stub_duration}s"
        )
        self._stub_timer = self.create_timer(
            self.pick_stub_duration,
            self._stub_pick_complete,
        )

    def _stub_pick_complete(self):
        if self._stub_timer is not None:
            self._stub_timer.cancel()
            self._stub_timer = None
        self.get_logger().info("[STUB] Pick complete — returning to IDLE")
        self._reset()

    # ── Reset ──────────────────────────────────────────────────────────────────
    def _reset(self):
        """
        Return cleanly to IDLE.
        Cancels any active Nav2 goal and disarms the stub timer.
        """
        if self._nav2_goal_handle is not None:
            self._nav2_goal_handle.cancel_goal_async()
            self._nav2_goal_handle = None

        if self._stub_timer is not None:
            self._stub_timer.cancel()
            self._stub_timer = None

        self._state           = State.IDLE
        self._target          = None
        self._nav2_retries    = 0
        self._verify_count    = 0
        self._arm_sequence    = []
        self._arm_step_index  = 0

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