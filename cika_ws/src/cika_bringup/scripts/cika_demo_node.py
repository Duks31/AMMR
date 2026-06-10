#!/usr/bin/env python3
"""
Cika Demo Script Node
---------------------
Publishes a pre-scripted /cmd_vel sequence to simulate autonomous
waste detection and sorting. Pauses at pick/drop points for manual
MoveIt execution.

Tune the values marked [TUNE] on the demo floor before the demo.
Run with:
    ros2 run cika_demo cika_demo_node
    (or: python3 cika_demo_node.py after sourcing workspace)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys


# ─────────────────────────────────────────────
#  TUNE THESE VALUES ON THE DEMO FLOOR
# ─────────────────────────────────────────────
DRIVE_SPEED       = 0.20   # [TUNE] m/s forward speed
TURN_SPEED        = 0.40   # [TUNE] rad/s rotation speed

# Object 1 (plastic) — distance from start to first object
DRIVE_TO_OBJ1_SEC = 3.5    # [TUNE] seconds to reach object 1

# Plastic bin — how far to turn and drive to reach it
TURN_TO_PLASTIC_BIN_SEC = 1.8   # [TUNE] rotate duration (right turn)
DRIVE_TO_PLASTIC_BIN_SEC = 2.0  # [TUNE] drive to bin

# Return to path — turn back and drive to object 2
TURN_BACK_FROM_PLASTIC_SEC = 1.8  # [TUNE] rotate back
DRIVE_TO_OBJ2_SEC = 2.5           # [TUNE] seconds to reach object 2

# Paper bin — how far to turn and drive to reach it
TURN_TO_PAPER_BIN_SEC = 1.8       # [TUNE] rotate duration (left turn)
DRIVE_TO_PAPER_BIN_SEC = 2.0      # [TUNE] drive to bin

SCAN_DELAY_SEC = 2.5  # fake scanning pause (looks good, don't touch)
# ─────────────────────────────────────────────


class CikaDemoNode(Node):

    def __init__(self):
        super().__init__('cika_demo_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Cika demo node ready.')

    # ── low-level movement primitives ────────

    def drive(self, speed: float, duration: float):
        """Drive forward (speed > 0) or backward (speed < 0)."""
        self._publish(linear_x=speed, duration=duration)

    def turn(self, angular_z: float, duration: float):
        """Turn in place. Positive = left (CCW), negative = right (CW)."""
        self._publish(angular_z=angular_z, duration=duration)

    def stop(self, pause: float = 0.5):
        """Publish zero velocity and wait briefly."""
        self._publish(linear_x=0.0, angular_z=0.0, duration=pause)

    def _publish(self, linear_x: float = 0.0, angular_z: float = 0.0,
                 duration: float = 1.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        end = time.time() + duration
        rate = self.create_rate(20)  # 20 Hz
        while time.time() < end:
            self.pub.publish(msg)
            rate.sleep()
        # Always publish a stop at end of move
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    # ── helper ───────────────────────────────

    def wait_for_enter(self, message: str):
        """Block until operator presses Enter."""
        self.stop(0.3)
        print(f'\n{"─"*55}')
        print(f'  ⏸  {message}')
        print(f'     Press ENTER when MoveIt is done...')
        print(f'{"─"*55}')
        input()
        print('  ▶  Continuing...\n')
        time.sleep(0.5)

    def announce(self, msg: str):
        self.get_logger().info(msg)
        print(f'\n  → {msg}')

    # ── main demo sequence ────────────────────

    def run_demo(self):
        time.sleep(1.0)  # let publisher settle
        print('\n' + '═'*55)
        print('  CIKA DEMO — waste sorting sequence')
        print('  Press Ctrl+C at any time to abort')
        print('═'*55 + '\n')
        input('  Press ENTER to start the demo...\n')

        # ── OBJECT 1: PLASTIC ─────────────────

        self.announce('Driving toward object 1 (plastic)...')
        self.drive(DRIVE_SPEED, DRIVE_TO_OBJ1_SEC)
        self.stop(0.8)

        self.announce(f'Scanning... ({SCAN_DELAY_SEC}s)')
        time.sleep(SCAN_DELAY_SEC)
        self.announce('Object detected: PLASTIC')

        self.wait_for_enter('Run MoveIt PICK for plastic object')

        self.announce('Turning toward plastic bin...')
        self.turn(-TURN_SPEED, TURN_TO_PLASTIC_BIN_SEC)   # right turn
        self.stop(0.3)
        self.drive(DRIVE_SPEED, DRIVE_TO_PLASTIC_BIN_SEC)
        self.stop(0.8)

        self.wait_for_enter('Run MoveIt DROP into plastic bin')

        # ── RETURN TO PATH ────────────────────

        self.announce('Returning to path...')
        self.turn(TURN_SPEED, TURN_BACK_FROM_PLASTIC_SEC)  # turn back left
        self.stop(0.3)

        # ── OBJECT 2: PAPER ───────────────────

        self.announce('Driving toward object 2 (paper)...')
        self.drive(DRIVE_SPEED, DRIVE_TO_OBJ2_SEC)
        self.stop(0.8)

        self.announce(f'Scanning... ({SCAN_DELAY_SEC}s)')
        time.sleep(SCAN_DELAY_SEC)
        self.announce('Object detected: PAPER')

        self.wait_for_enter('Run MoveIt PICK for paper object')

        self.announce('Turning toward paper bin...')
        self.turn(TURN_SPEED, TURN_TO_PAPER_BIN_SEC)   # left turn
        self.stop(0.3)
        self.drive(DRIVE_SPEED, DRIVE_TO_PAPER_BIN_SEC)
        self.stop(0.8)

        self.wait_for_enter('Run MoveIt DROP into paper bin')

        # ── DONE ──────────────────────────────

        self.stop(0.5)
        print('\n' + '═'*55)
        print('  ✓  Demo complete!')
        print('═'*55 + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = CikaDemoNode()
    try:
        node.run_demo()
    except KeyboardInterrupt:
        print('\n  Demo aborted by operator.')
        # Emergency stop
        twist = Twist()
        node.pub.publish(twist)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()