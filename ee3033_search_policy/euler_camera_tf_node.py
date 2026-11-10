"""Broadcast a pitch/roll-aware camera body frame for the balancing robot."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def quaternion_from_rpy(roll, pitch, yaw=0.0):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class EulerCameraTf(Node):
    """Use robot roll/pitch without applying Euler yaw a second time."""

    def __init__(self):
        super().__init__("euler_camera_tf")
        defaults = {
            "euler_topic": "/robot_euler",
            "parent_frame": "base_footprint",
            "child_frame": "ee3033_balance_body",
            "expected_source_frame": "base_footprint",
            "roll_scale": 1.0,
            "pitch_scale": 1.0,
            "roll_offset_rad": 0.0,
            "pitch_offset_rad": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.parent = str(p("parent_frame"))
        self.child = str(p("child_frame"))
        self.expected = str(p("expected_source_frame"))
        self.roll_scale = float(p("roll_scale"))
        self.pitch_scale = float(p("pitch_scale"))
        self.roll_offset = float(p("roll_offset_rad"))
        self.pitch_offset = float(p("pitch_offset_rad"))
        values = (
            self.roll_scale,
            self.pitch_scale,
            self.roll_offset,
            self.pitch_offset,
        )
        if not self.parent or not self.child or not all(math.isfinite(v) for v in values):
            raise ValueError("Invalid Euler camera TF parameters")
        self.broadcaster = TransformBroadcaster(self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Vector3Stamped, str(p("euler_topic")), self._euler, qos
        )

    def _euler(self, msg):
        if self.expected and msg.header.frame_id != self.expected:
            self.get_logger().warning(
                "Rejected robot_euler frame " + repr(msg.header.frame_id),
                throttle_duration_sec=2.0,
            )
            return
        raw = (msg.vector.x, msg.vector.y, msg.vector.z)
        if not all(math.isfinite(v) for v in raw):
            return
        roll = self.roll_scale * msg.vector.x + self.roll_offset
        pitch = self.pitch_scale * msg.vector.y + self.pitch_offset
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, 0.0)
        tf = TransformStamped()
        tf.header = msg.header
        tf.header.frame_id = self.parent
        tf.child_frame_id = self.child
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = EulerCameraTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
