#!/usr/bin/env python3

"""
Simple coordinate navigation controller for a Kiwi omnidirectional robot.

Inputs:
    /goal_pose   geometry_msgs/PoseStamped
    /scan        sensor_msgs/LaserScan
    TF:          map -> base_link

Output:
    /cmd_vel     geometry_msgs/Twist

Usage:

    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'},
      pose: {position: {x: 3.0, y: 2.0, z: 0.0},
             orientation: {w: 1.0}}}"

The robot:
    1. Gets its pose from SLAM TF.
    2. Calculates the vector to the goal.
    3. Converts that vector into the robot frame.
    4. Uses Kiwi X/Y motion to move toward the goal.
    5. Uses LiDAR for simple local obstacle avoidance.
    6. Stops when it reaches the goal.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan

import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class NavigationController(Node):

    def __init__(self):
        super().__init__('navigation_controller')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('strafe_speed', 0.35)
        self.declare_parameter('angular_speed', 0.6)

        self.declare_parameter('goal_tolerance', 0.20)

        self.declare_parameter('obstacle_distance', 0.65)
        self.declare_parameter('emergency_distance', 0.35)

        self.declare_parameter('control_rate', 10.0)

        goal_topic = self.get_parameter('goal_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.linear_speed = self.get_parameter('linear_speed').value
        self.strafe_speed = self.get_parameter('strafe_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        self.goal_tolerance = self.get_parameter(
            'goal_tolerance'
        ).value

        self.obstacle_distance = self.get_parameter(
            'obstacle_distance'
        ).value

        self.emergency_distance = self.get_parameter(
            'emergency_distance'
        ).value

        control_rate = self.get_parameter(
            'control_rate'
        ).value

        # ---------------------------------------------------------
        # ROS publishers/subscribers
        # ---------------------------------------------------------

        self.cmd_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            goal_topic,
            self.goal_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )

        # ---------------------------------------------------------
        # TF
        # ---------------------------------------------------------

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # ---------------------------------------------------------
        # State
        # ---------------------------------------------------------

        self.scan_data = None

        self.goal_x = None
        self.goal_y = None

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        self.goal_active = False

        # Hysteresis state — prevents command chattering ("fidgeting")
        # when front distance or heading error hovers right at a
        # single hard threshold.
        self.avoiding_obstacle = False
        self.correcting_heading = False

        # ---------------------------------------------------------
        # Control timer
        # ---------------------------------------------------------

        self.timer = self.create_timer(
            1.0 / control_rate,
            self.control_loop
        )

        self.get_logger().info(
            'Simple Kiwi navigation controller started.'
        )

        self.get_logger().info(
            f'Waiting for goals on {goal_topic}'
        )

    # =============================================================
    # CALLBACKS
    # =============================================================

    def goal_callback(self, msg):

        # Goal is expected in map frame.
        #
        # If another frame is supplied, transform it into map.

        if msg.header.frame_id and \
                msg.header.frame_id != self.map_frame:

            try:

                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    msg.header.frame_id,
                    rclpy.time.Time()
                )

                transformed = do_transform_pose(
                    msg.pose,
                    transform
                )

                self.goal_x = transformed.position.x
                self.goal_y = transformed.position.y

            except Exception as e:

                self.get_logger().warn(
                    f'Could not transform goal: {e}'
                )

                return

        else:

            self.goal_x = msg.pose.position.x
            self.goal_y = msg.pose.position.y

        self.goal_active = True

        self.get_logger().info(
            f'New goal: '
            f'x={self.goal_x:.2f}, '
            f'y={self.goal_y:.2f}'
        )

    def scan_callback(self, msg):

        self.scan_data = msg

    # =============================================================
    # ROBOT POSE
    # =============================================================

    def update_robot_pose(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )

            t = transform.transform.translation
            q = transform.transform.rotation

            self.robot_x = t.x
            self.robot_y = t.y

            # Quaternion -> yaw

            sin_yaw = 2.0 * (
                q.w * q.z +
                q.x * q.y
            )

            cos_yaw = 1.0 - 2.0 * (
                q.y * q.y +
                q.z * q.z
            )

            self.robot_yaw = math.atan2(
                sin_yaw,
                cos_yaw
            )

            return True

        except Exception:

            return False

    # =============================================================
    # LIDAR
    # =============================================================

    def get_sector_min(
        self,
        angle_min,
        angle_max
    ):

        if self.scan_data is None:
            return float('inf')

        msg = self.scan_data

        minimum = float('inf')

        angle = msg.angle_min

        for r in msg.ranges:

            if angle_min <= angle <= angle_max:

                if (
                    math.isfinite(r)
                    and r > msg.range_min
                    and r < msg.range_max
                ):

                    minimum = min(
                        minimum,
                        r
                    )

            angle += msg.angle_increment

        return minimum

    # =============================================================
    # STOP
    # =============================================================

    def stop_robot(self):

        self.cmd_pub.publish(Twist())

    # =============================================================
    # MAIN CONTROL
    # =============================================================

    def control_loop(self):

        # No goal yet.
        if not self.goal_active:
            return

        # Need LiDAR.
        if self.scan_data is None:
            return

        # Need SLAM pose.
        if not self.update_robot_pose():
            return

        # ---------------------------------------------------------
        # Distance to goal
        # ---------------------------------------------------------

        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y

        distance = math.hypot(
            dx,
            dy
        )

        # ---------------------------------------------------------
        # Goal reached
        # ---------------------------------------------------------

        if distance <= self.goal_tolerance:

            self.stop_robot()

            self.get_logger().info(
                f'Goal reached! '
                f'({self.robot_x:.2f}, '
                f'{self.robot_y:.2f})'
            )

            self.goal_active = False

            return

        # ---------------------------------------------------------
        # Convert goal from MAP frame to ROBOT frame
        # ---------------------------------------------------------

        cos_yaw = math.cos(
            self.robot_yaw
        )

        sin_yaw = math.sin(
            self.robot_yaw
        )

        goal_x_robot = (
            cos_yaw * dx +
            sin_yaw * dy
        )

        goal_y_robot = (
            -sin_yaw * dx +
            cos_yaw * dy
        )

        # ---------------------------------------------------------
        # LIDAR sectors
        # ---------------------------------------------------------

        front = self.get_sector_min(
            math.radians(-30),
            math.radians(30)
        )

        left = self.get_sector_min(
            math.radians(45),
            math.radians(120)
        )

        right = self.get_sector_min(
            math.radians(-120),
            math.radians(-45)
        )

        front_left = self.get_sector_min(
            math.radians(20),
            math.radians(90)
        )

        front_right = self.get_sector_min(
            math.radians(-90),
            math.radians(-20)
        )

        cmd = Twist()

        # =========================================================
        # EMERGENCY
        # =========================================================

        if front < self.emergency_distance:

            self.get_logger().warn(
                f'Emergency obstacle: '
                f'{front:.2f}m'
            )

            # Back away slightly.
            cmd.linear.x = -0.15

            self.cmd_pub.publish(cmd)

            return

        # =========================================================
        # OBSTACLE AHEAD (hysteresis: enter/exit at different
        # distances so `front` hovering near one threshold doesn't
        # flip the command back and forth every tick)
        # =========================================================

        if front < self.obstacle_distance:
            self.avoiding_obstacle = True
        elif front > self.obstacle_distance * 1.3:
            self.avoiding_obstacle = False

        if self.avoiding_obstacle:

            # Move sideways toward the side with more clearance.

            if left > right:

                cmd.linear.y = self.strafe_speed

            else:

                cmd.linear.y = -self.strafe_speed

            self.cmd_pub.publish(cmd)

            return

        # =========================================================
        # GO DIRECTLY TOWARD GOAL
        # =========================================================

        magnitude = math.hypot(
            goal_x_robot,
            goal_y_robot
        )

        if magnitude > 0:

            direction_x = (
                goal_x_robot /
                magnitude
            )

            direction_y = (
                goal_y_robot /
                magnitude
            )

        else:

            direction_x = 0.0
            direction_y = 0.0

        # ---------------------------------------------------------
        # Kiwi omnidirectional movement
        # ---------------------------------------------------------

        cmd.linear.x = (
            direction_x *
            self.linear_speed
        )

        cmd.linear.y = (
            direction_y *
            self.strafe_speed
        )

        # ---------------------------------------------------------
        # Limit combined velocity
        # ---------------------------------------------------------

        velocity = math.hypot(
            cmd.linear.x,
            cmd.linear.y
        )

        max_velocity = self.linear_speed

        if velocity > max_velocity:

            scale = (
                max_velocity /
                velocity
            )

            cmd.linear.x *= scale
            cmd.linear.y *= scale

        # ---------------------------------------------------------
        # Rotate toward goal if badly misaligned.
        #
        # Kiwi can translate independently, so this is deliberately
        # mild rather than making rotation mandatory.
        # ---------------------------------------------------------

        desired_heading = math.atan2(
            dy,
            dx
        )

        heading_error = (
            desired_heading -
            self.robot_yaw
        )

        heading_error = math.atan2(
            math.sin(heading_error),
            math.cos(heading_error)
        )

        # Hysteresis + proportional scaling: snapping angular.z between
        # 0 and full speed at one exact heading_error threshold makes
        # the command chatter as heading_error hovers right at that
        # boundary. Start correcting above 70 deg, keep correcting
        # until it drops below 35 deg (wide deadband gap), and ease
        # the command in/out proportionally instead of a hard jump.
        enter_threshold = math.radians(70)
        exit_threshold = math.radians(35)

        if abs(heading_error) > enter_threshold:
            self.correcting_heading = True
        elif abs(heading_error) < exit_threshold:
            self.correcting_heading = False

        if self.correcting_heading:

            angular_scale = min(1.0, abs(heading_error) / enter_threshold)

            cmd.angular.z = (
                self.angular_speed * angular_scale
                if heading_error > 0
                else -self.angular_speed * angular_scale
            )

        else:

            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # =============================================================
    # SHUTDOWN
    # =============================================================

    def destroy_node(self):

        self.stop_robot()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = NavigationController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()