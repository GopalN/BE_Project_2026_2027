#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

MIN_STATE_TICKS = 15   # at 10 Hz control loop -> 1.5s minimum dwell in ROTATE/STRAFE
                      
CLEAR_STREAK_NEEDED = 4  # consecutive clear ticks required before trusting it
                         
ROTATE_TIMEOUT_TICKS = 40  # 4s — if still rotating this long, likely stuck in cornes
                            
BACKUP_TICKS = 10          # 1s of reversing to actually gain clearance


class AutonomousController(Node):
    def __init__(self):
        super().__init__('autonomous_controller')

        self.declare_parameter('obstacle_distance', 1.1)
        self.declare_parameter('side_obstacle_distance', 0.9)
        self.declare_parameter('linear_speed', 0.4)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('strafe_speed', 0.3)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.side_obstacle_distance = self.get_parameter('side_obstacle_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.strafe_speed = self.get_parameter('strafe_speed').value
        scan_topic = self.get_parameter('scan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10
        )

        self.state = 'FORWARD'
        self.rotate_direction = 1.0
        self.scan_data = None
        self.state_ticks = 0  # how many ticks we've been in current state
        self.clear_streak = 0  # consecutive ticks the front sector has read clear

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Autonomous controller started')
        self.get_logger().info(
            f'  obstacle_distance={self.obstacle_distance}m, '
            f'linear_speed={self.linear_speed}m/s, '
            f'angular_speed={self.angular_speed}rad/s'
        )

    def scan_callback(self, msg: LaserScan):
        self.scan_data = msg

    def get_sector_min(self, ranges, angle_min, angle_max, msg: LaserScan):
        if not ranges:
            return float('inf')
        start_idx = max(0, int((angle_min - msg.angle_min) / msg.angle_increment))
        end_idx = min(len(ranges) - 1, int((angle_max - msg.angle_min) / msg.angle_increment))
        if start_idx > end_idx:
            return float('inf')
        sector = ranges[start_idx:end_idx + 1]
        valid = [r for r in sector if msg.range_min < r < msg.range_max]
        return min(valid) if valid else float('inf')

    def set_state(self, new_state, rotate_direction=None):
        """Switch state and reset the dwell counter."""
        self.state = new_state
        self.state_ticks = 0
        self.clear_streak = 0
        if rotate_direction is not None:
            self.rotate_direction = rotate_direction

    def control_loop(self):
        if self.scan_data is None:
            return

        msg = self.scan_data
        ranges = list(msg.ranges)

        front_min = self.get_sector_min(ranges, math.radians(-30), math.radians(30), msg)
        front_left_min = self.get_sector_min(ranges, math.radians(30), math.radians(90), msg)
        front_right_min = self.get_sector_min(ranges, math.radians(-90), math.radians(-30), msg)
        left_min = self.get_sector_min(ranges, math.radians(60), math.radians(120), msg)
        right_min = self.get_sector_min(ranges, math.radians(-120), math.radians(-60), msg)

        cmd = Twist()

        
        if front_min > self.obstacle_distance * 1.2: # tracking consecutive clear readings; only trusting clear once it's held steady for several ticks, not on a single instanec.
            self.clear_streak += 1
        else:
            self.clear_streak = 0

        can_switch = (
            self.state_ticks >= MIN_STATE_TICKS
            and self.clear_streak >= CLEAR_STREAK_NEEDED
        )

        if self.state == 'FORWARD':
            if front_min < self.obstacle_distance:
                if left_min > self.side_obstacle_distance and right_min <= self.side_obstacle_distance:
                    self.set_state('STRAFE_LEFT')
                    self.get_logger().info(
                        f'Obstacle at {front_min:.2f}m — strafing left', throttle_duration_sec=1.0
                    )
                    cmd.linear.y = self.strafe_speed
                elif right_min > self.side_obstacle_distance and left_min <= self.side_obstacle_distance:
                    self.set_state('STRAFE_RIGHT')
                    self.get_logger().info(
                        f'Obstacle at {front_min:.2f}m — strafing right', throttle_duration_sec=1.0
                    )
                    cmd.linear.y = -self.strafe_speed
                else:
                    direction = 1.0 if front_left_min > front_right_min else -1.0
                    self.set_state('ROTATE', rotate_direction=direction)
                    self.get_logger().info(
                        f'Obstacle at {front_min:.2f}m — rotating', throttle_duration_sec=1.0
                    )
                    cmd.angular.z = self.angular_speed * self.rotate_direction
            else:
                cmd.linear.x = self.linear_speed

        elif self.state == 'ROTATE':
            if can_switch:
                self.set_state('FORWARD')
                self.get_logger().info('Path clear — resuming forward', throttle_duration_sec=1.0)
                cmd.linear.x = self.linear_speed
            elif self.state_ticks >= ROTATE_TIMEOUT_TICKS:
                # Likely boxed in (e.g. a corner) — no heading found enough
                # clearance. Back up to actually gain space instead of
                # spinning indefinitely.
                self.set_state('BACKUP')
                self.get_logger().info(
                    'Rotation timed out — backing up', throttle_duration_sec=1.0
                )
                cmd.linear.x = -self.linear_speed
            else:
                cmd.angular.z = self.angular_speed * self.rotate_direction

        elif self.state == 'BACKUP':
            if self.state_ticks >= BACKUP_TICKS: # Re-evaluate and try rotating again with fresh readings.
                direction = 1.0 if front_left_min > front_right_min else -1.0
                self.set_state('ROTATE', rotate_direction=direction)
                cmd.angular.z = self.angular_speed * self.rotate_direction
            else:
                cmd.linear.x = -self.linear_speed

        elif self.state == 'STRAFE_LEFT':
            if can_switch:
                self.set_state('FORWARD')
                self.get_logger().info('Path clear — resuming forward', throttle_duration_sec=1.0)
                cmd.linear.x = self.linear_speed
            elif left_min < self.side_obstacle_distance:
                self.set_state('ROTATE', rotate_direction=-1.0)
                cmd.angular.z = self.angular_speed * self.rotate_direction
            else:
                cmd.linear.y = self.strafe_speed

        elif self.state == 'STRAFE_RIGHT':
            if can_switch:
                self.set_state('FORWARD')
                self.get_logger().info('Path clear — resuming forward', throttle_duration_sec=1.0)
                cmd.linear.x = self.linear_speed
            elif right_min < self.side_obstacle_distance:
                self.set_state('ROTATE', rotate_direction=1.0)
                cmd.angular.z = self.angular_speed * self.rotate_direction
            else:
                cmd.linear.y = -self.strafe_speed

        self.state_ticks += 1
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
