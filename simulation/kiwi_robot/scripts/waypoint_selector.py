#!/usr/bin/env python3

"""
Interactive waypoint selector + navigation controller for the Kiwi
omnidirectional warehouse robot.

This is a single-process pipeline:

    1. Presents an interactive menu of destinations (aisles + conveyor)
       on stdin.
    2. When the user picks one, publishes a PoseStamped on /goal_pose
       AND drives the robot to it in the same process using TF-based
       localization, LiDAR obstacle avoidance and Kiwi omnidirectional
       velocity commands on /cmd_vel.
    3. Stops the robot when the goal is reached, then re-prompts the
       menu for the next destination.

Inputs:
    /scan        sensor_msgs/LaserScan
    TF:          map -> base_link

Outputs:
    /goal_pose   geometry_msgs/PoseStamped   (for logging / external tools)
    /cmd_vel     geometry_msgs/Twist

Usage:
    ros2 run kiwi_robot waypoint_selector.py
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import tf2_ros
from tf2_geometry_msgs import do_transform_pose


# Standoff points derived from kiwi_world.sdf model origins:
#   aisle_1_shelf_*  x=-7   -> stand 1.5m east in the corridor, y at the
#                              aisle's middle shelf (y=-1)
#   aisle_2_shelf_*  x=-3   -> stand 1.5m east
#   aisle_3_shelf_*  x=1    -> stand 1.5m east
#   conveyor_belt_1  x=6,y=0 -> stand 1.5m west, facing the belt
DESTINATIONS = {
    '1': {'label': 'Aisle 1', 'x': -5.5, 'y': -1.0},
    '2': {'label': 'Aisle 2', 'x': -1.5, 'y': -1.0},
    '3': {'label': 'Aisle 3', 'x': 2.5, 'y': -1.0},
    '4': {'label': 'Conveyor Belt', 'x': 4.5, 'y': 0.0},
}

# Maps external GUI/RViz panel destination identifiers to the internal
# DESTINATIONS dictionary keys used by the terminal menu.
GUI_DESTINATION_MAP = {
    'aisle_1': '1',
    'aisle_2': '2',
    'aisle_3': '3',
    'conveyor_belt': '4',
}


class WaypointSelector(Node):

    def __init__(self):
        super().__init__('waypoint_selector')

        # -----------------------------------------------------------
        # Parameters
        # -----------------------------------------------------------
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

        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.emergency_distance = self.get_parameter('emergency_distance').value

        control_rate = self.get_parameter('control_rate').value

        # -----------------------------------------------------------
        # ROS publishers / subscribers
        # -----------------------------------------------------------
        # Reentrant so the control timer and the scan callback can run
        # concurrently with the (blocking) menu thread's occasional
        # spin_once-style publishes.
        cb_group = ReentrantCallbackGroup()

        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10,
            callback_group=cb_group,
        )
        self.create_subscription(
            String, '/waypoint_selector/select_destination',
            self.gui_destination_callback, 10,
        )

        # -----------------------------------------------------------
        # TF
        # -----------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -----------------------------------------------------------
        # State
        # -----------------------------------------------------------
        self.scan_data = None
        self._lidar_disabled_warned = False

        self.goal_x = None
        self.goal_y = None
        self.goal_active = False
        self._goal_lock = threading.Lock()

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        self.start_time = self.get_clock().now()

        # -----------------------------------------------------------
        # Control timer
        # -----------------------------------------------------------
        self.timer = self.create_timer(
            1.0 / control_rate,
            self.control_loop,
            callback_group=cb_group,
        )

        self.get_logger().info('Waypoint selector + navigation controller ready.')

    # =================================================================
    # GOAL PUBLISHING (from menu)
    # =================================================================

    def set_goal(self, x, y, label):
        with self._goal_lock:
            self.goal_x = float(x)
            self.goal_y = float(y)
            self.goal_active = True

        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

        self.get_logger().info(
            f'Sending robot to {label} (x={x:.2f}, y={y:.2f})'
        )
    # GUI/RVIZ DESTINATION SELECTION
    # =================================================================

    def gui_destination_callback(self, msg):
        key = GUI_DESTINATION_MAP.get(msg.data)
        dest = DESTINATIONS.get(key) if key is not None else None

        if dest is None:
            self.get_logger().warn(
                f"Unknown destination '{msg.data}' received on "
                f"/waypoint_selector/select_destination. Valid options: "
                f"{list(GUI_DESTINATION_MAP.keys())}"
            )
            return

        self.get_logger().info(
            f"GUI selected destination: {msg.data} -> {dest['label']}"
        )
        self.set_goal(dest['x'], dest['y'], dest['label'])

    # =================================================================

    # =================================================================
    # CALLBACKS
    # =================================================================

    def scan_callback(self, msg):
        self.scan_data = msg

    # =================================================================
    # ROBOT POSE
    # =================================================================

    def update_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
            )

            t = transform.transform.translation
            q = transform.transform.rotation

            self.robot_x = t.x
            self.robot_y = t.y

            sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
            cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(sin_yaw, cos_yaw)
            return True
        except Exception:
            return False

    # =================================================================
    # LIDAR
    # =================================================================

    def get_sector_min(self, angle_min, angle_max):
        if self.scan_data is None:
            return float('inf')

        msg = self.scan_data
        minimum = float('inf')
        angle = msg.angle_min

        for r in msg.ranges:
            if angle_min <= angle <= angle_max:
                if (math.isfinite(r)
                        and r > msg.range_min
                        and r < msg.range_max):
                    minimum = min(minimum, r)
            angle += msg.angle_increment

        return minimum

    # =================================================================
    # STOP
    # =================================================================

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    # =================================================================
    # MAIN CONTROL LOOP
    # =================================================================

    def control_loop(self):
        with self._goal_lock:
            if not self.goal_active:
                return
            goal_x = self.goal_x
            goal_y = self.goal_y

        if self.scan_data is None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed < 5.0:
                self.get_logger().warn(
                    'No scan data received yet', throttle_duration_sec=2.0
                )
                return
            else:
                if not self._lidar_disabled_warned:
                    self.get_logger().warn(
                        'No scan data received after 5s; proceeding with '
                        'TF-based navigation without LiDAR obstacle avoidance.'
                    )
                    self._lidar_disabled_warned = True
        else:
            self._lidar_disabled_warned = False

        if not self.update_robot_pose():
            self.get_logger().warn(
                'TF lookup map->base_link failed, cannot compute drive command',
                throttle_duration_sec=2.0,
            )
            return

        dx = goal_x - self.robot_x
        dy = goal_y - self.robot_y
        distance = math.hypot(dx, dy)

        if distance <= self.goal_tolerance:
            self.stop_robot()
            self.get_logger().info(
                f'Goal reached! ({self.robot_x:.2f}, {self.robot_y:.2f})'
            )
            with self._goal_lock:
                self.goal_active = False
            return

        # Map -> robot frame
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        goal_x_robot = cos_yaw * dx + sin_yaw * dy
        goal_y_robot = -sin_yaw * dx + cos_yaw * dy

        # LiDAR sectors
        front = self.get_sector_min(math.radians(-30), math.radians(30))
        left = self.get_sector_min(math.radians(45), math.radians(120))
        right = self.get_sector_min(math.radians(-120), math.radians(-45))

        cmd = Twist()

        # Emergency reverse
        if front < self.emergency_distance:
            self.get_logger().warn(
                f'Emergency obstacle: {front:.2f}m', throttle_duration_sec=1.0
            )
            cmd.linear.x = -0.15
            self.cmd_pub.publish(cmd)
            return

        # Obstacle ahead -> strafe
        if front < self.obstacle_distance:
            if left > right:
                cmd.linear.y = self.strafe_speed
            else:
                cmd.linear.y = -self.strafe_speed
            self.cmd_pub.publish(cmd)
            return

        # Go directly toward goal
        magnitude = math.hypot(goal_x_robot, goal_y_robot)
        if magnitude > 0:
            direction_x = goal_x_robot / magnitude
            direction_y = goal_y_robot / magnitude
        else:
            direction_x = 0.0
            direction_y = 0.0

        cmd.linear.x = direction_x * self.linear_speed
        cmd.linear.y = direction_y * self.strafe_speed

        velocity = math.hypot(cmd.linear.x, cmd.linear.y)
        max_velocity = self.linear_speed
        if velocity > max_velocity:
            scale = max_velocity / velocity
            cmd.linear.x *= scale
            cmd.linear.y *= scale

        # Mild heading correction
        desired_heading = math.atan2(dy, dx)
        heading_error = desired_heading - self.robot_yaw
        heading_error = math.atan2(
            math.sin(heading_error), math.cos(heading_error)
        )
        if abs(heading_error) > math.radians(70):
            cmd.angular.z = (
                self.angular_speed if heading_error > 0 else -self.angular_speed
            )
        else:
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # =================================================================
    # SHUTDOWN
    # =================================================================

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()


# ---------------------------------------------------------------------
# Interactive menu (runs on its own thread so ROS spinning is unblocked)
# ---------------------------------------------------------------------

def menu_loop(node: WaypointSelector, shutdown_event: threading.Event):
    while not shutdown_event.is_set() and rclpy.ok():
        print('\nSelect a destination:')
        for key, dest in DESTINATIONS.items():
            print(f'  {key}) {dest["label"]}')
        print('  q) Quit')

        try:
            choice = input('Enter choice: ').strip()
        except EOFError:
            break

        if choice.lower() == 'q':
            break

        dest = DESTINATIONS.get(choice)
        if dest is None:
            print('Invalid choice, try again.')
            continue

        node.set_goal(dest['x'], dest['y'], dest['label'])

    shutdown_event.set()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSelector()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    shutdown_event = threading.Event()
    menu_thread = threading.Thread(
        target=menu_loop, args=(node, shutdown_event), daemon=True
    )
    menu_thread.start()

    try:
        while rclpy.ok() and not shutdown_event.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_event.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
