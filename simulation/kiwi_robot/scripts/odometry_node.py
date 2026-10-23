#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class KiwiOdometry(Node):

    def __init__(self):
        super().__init__('kiwi_odometry')

        # Kiwi robot geometry
        self.wheel_radius = 0.08   # metres
        self.wheel_offset = 0.24   # metres

        # Robot pose in odom frame
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Current body velocity
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # Time
        self.last_time = self.get_clock().now()

        
        self.front_wheel = 0.0          # Wheel angular velocities
        self.rear_left_wheel = 0.0
        self.rear_right_wheel = 0.0

        # Subscribe to measured wheel states
        self.joint_sub = self.create_subscription(
            JointState,
            '/model/kiwi_robot/joint_state',
            self.joint_state_callback,
            10
        )

        # Publish odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

       
        self.tf_broadcaster = TransformBroadcaster(self)  # Publish odom -> base_link transform

       
        self.timer = self.create_timer(        # Run odometry at 50 Hz
            0.02,
            self.update_odometry
        )

        self.get_logger().info('Kiwi manual odometry started')


    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):

            if i >= len(msg.velocity):
                continue

            velocity = msg.velocity[i]

            if name == 'front_wheel_joint':
                self.front_wheel = velocity

            elif name == 'rear_left_wheel_joint':
                self.rear_left_wheel = velocity

            elif name == 'rear_right_wheel_joint':
                self.rear_right_wheel = velocity


    def calculate_body_velocity(self):
        r = self.wheel_radius
        L = self.wheel_offset

        wf = self.front_wheel
        wrl = self.rear_left_wheel
        wrr = self.rear_right_wheel
        self.vx = r * (
            -wf +
            0.5 * wrl +
            0.5 * wrr
        )

        self.vy = (r / math.sqrt(3.0)) * (
            -wf -
            0.5 * wrl +
            1.5 * wrr
        )

        self.wz = (r / (2.0 * L)) * (
            wrl + wrr
        )


    def update_odometry(self):

        current_time = self.get_clock().now()

        dt = (
            current_time - self.last_time
        ).nanoseconds / 1e9

        self.last_time = current_time

        # Ignore invalid time steps
        if dt <= 0.0 or dt > 1.0:
            return

        self.calculate_body_velocity()      # Calculate robot velocity from wheel velocities

        cos_theta = math.cos(self.theta)   # Body-frame velocity -> odom/world-frame velocity
        sin_theta = math.sin(self.theta)

        world_vx = (
            self.vx * cos_theta -
            self.vy * sin_theta
        )

        world_vy = (
            self.vx * sin_theta +
            self.vy * cos_theta
        )

        
        self.x += world_vx * dt       # Integrate velocity to obtain pose
        self.y += world_vy * dt
        self.theta += self.wz * dt

       
        self.theta = math.atan2(      # Keep theta within [-pi, pi]
            math.sin(self.theta),
            math.cos(self.theta)
        )

        self.publish_odometry(current_time)


    def publish_odometry(self, current_time):

        timestamp = current_time.to_msg()

        odom = Odometry()               # Publish Odometry message

        odom.header.stamp = timestamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation
        odom.pose.pose.orientation.z = math.sin(
            self.theta / 2.0
        )

        odom.pose.pose.orientation.w = math.cos(
            self.theta / 2.0
        )

        # Velocity in robot frame
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz

        self.odom_pub.publish(odom)

        # Publish TF      
        transform = TransformStamped()

        transform.header.stamp = timestamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.z = math.sin(
            self.theta / 2.0
        )

        transform.transform.rotation.w = math.cos(
            self.theta / 2.0
        )

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):

    rclpy.init(args=args)

    node = KiwiOdometry()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

