#!/usr/bin/env python3
"""
serial_bridge.py
Reads ESP32 serial output and publishes sensor_msgs/Imu to /imu/raw.
Encoder odometry is handled by ros2_control — we only care about IMU lines here
Reads ESP32 serial output and publishes:
- sensor_msgs/Imu to /imu/raw
- sensor_msgs/MagneticField to /imu/mag
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import serial


class SerialBridge(Node):
    def __init__(self):
        super().__init__("serial_bridge")

        self.declare_parameter("serial_port", "/dev/ttyUSB1")
        self.declare_parameter("baud_rate", 115200)

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = self.get_parameter("baud_rate").get_parameter_value().integer_value

        self.imu_pub = self.create_publisher(Imu, "/imu/raw", 10)
        self.mag_pub = self.create_publisher(
            MagneticField, "/imu/mag", 10
        )  # <-- New Publisher

        # Publish static transform: base_link → imu_link
        self._tf_static = StaticTransformBroadcaster(self)
        self._publish_imu_tf()

        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f"Serial bridge open on {port} at {baud}")
        except serial.SerialException as e:
            self.get_logger().error(f"Cannot open serial port: {e}")
            raise

        self.create_timer(0.001, self.read_serial)  # poll at 1000Hz, non-blocking

    def _publish_imu_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "imu_link"
        # Adjust x,y,z to where the GY-87 is physically mounted on cika
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.05
        t.transform.rotation.w = 1.0
        self._tf_static.sendTransform(t)

    def read_serial(self):
        if not self.ser.in_waiting:
            return
        try:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
        except serial.SerialException:
            return

        if line.startswith("I:"):
            self._handle_imu(line[2:])
        # E: lines are ignored here — ros2_control handles encoder odometry


    def _handle_imu(self, data: str):
        try:
            vals = list(map(float, data.split(",")))
            # 1. Unpack all 9 values
            ax, ay, az, gx, gy, gz, mx, my, mz = vals[0:9]
        except (ValueError, IndexError):
            self.get_logger().warn(f"Bad IMU line: {data}")
            return

        # 2. Grab the exact time ONCE so both messages are perfectly synced
        current_time = self.get_clock().now().to_msg()

        # --- IMU Message ---
        msg = Imu()
        msg.header.stamp = current_time
        msg.header.frame_id = "imu_link"

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # No orientation computed here — Madgwick handles that
        msg.orientation_covariance[0] = -1.0

        # Covariances — tune these after testing
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01
        msg.angular_velocity_covariance[0] = 0.005
        msg.angular_velocity_covariance[4] = 0.005
        msg.angular_velocity_covariance[8] = 0.005

        self.imu_pub.publish(msg)

        # --- Magnetic Field Message ---
        # Only publish if mag data is valid (not 0.0 from an error)
        if mx != 0.0 or my != 0.0 or mz != 0.0:
            mag_msg = MagneticField()
            mag_msg.header.stamp = current_time  # MUST match the IMU timestamp
            mag_msg.header.frame_id = "imu_link"  # MUST match the IMU frame_id

            # Convert µT to Teslas (T)
            mag_msg.magnetic_field.x = mx * 1e-6
            mag_msg.magnetic_field.y = my * 1e-6
            mag_msg.magnetic_field.z = mz * 1e-6

            # Basic covariance matrix
            mag_msg.magnetic_field_covariance[0] = 1e-6
            mag_msg.magnetic_field_covariance[4] = 1e-6
            mag_msg.magnetic_field_covariance[8] = 1e-6

            self.mag_pub.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
