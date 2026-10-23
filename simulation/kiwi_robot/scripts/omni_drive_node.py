#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

WHEEL_RADIUS = 0.08
WHEEL_OFFSET = 0.24  # L: distance from robot center to wheel contact

WHEELS = {
    'front':      math.radians(90),
    'rear_left':  math.radians(210),
    'rear_right': math.radians(-30),
}

# Flip to -1.0 for any wheel that spins backwards in testing.
WHEEL_SIGN = {
    'front': 1.0,
    'rear_left': 1.0,
    'rear_right': 1.0,
}


class OmniDriveNode(Node):
    def __init__(self):
        super().__init__('omni_drive_node')

        self.publishers_map = {
            name: self.create_publisher(
                Float64, f'/model/kiwi_robot/joint/{name}_wheel_joint/cmd_vel', 10
            )
            for name in WHEELS
        }

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        self.get_logger().info('omni_drive_node started — converting cmd_vel to 3 wheel speeds')

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        for name, theta in WHEELS.items():
            speed = (-math.sin(theta) * vx + math.cos(theta) * vy + WHEEL_OFFSET * wz) / WHEEL_RADIUS
            speed *= WHEEL_SIGN[name]

            out = Float64()
            out.data = speed
            self.publishers_map[name].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OmniDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally: 
        stop = Float64() # stop all wheels on shutdown
        stop.data = 0.0
        for pub in node.publishers_map.values():
            pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
