import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('kiwi_robot')

    # Process xacro
    urdf_file = os.path.join(pkg_share, 'urdf', 'kiwi_robot.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    # World file
    world_file = os.path.join(pkg_share, 'worlds', 'kiwi_world.sdf')

    # Use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
    )

    # Spawn robot (delayed to let Gazebo start)
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', '/robot_description',
                    '-name', 'kiwi_robot',
                    '-z', '0.0',
                ],
                output='screen',
            ),
        ],
    )

    # Bridge: clock, odom, scan, joint_states, tf, and the three per-wheel
    # velocity commands the JointController plugins actually listen to.
    # (The generic /cmd_vel Twist bridge is kept out on purpose — nothing on
    # the Gazebo side subscribes to a Twist; this is an omni/kiwi drive
    # controlled per-wheel via Float64 -> gz.msgs.Double topics.)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/kiwi_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/kiwi_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/kiwi_robot/joint/front_wheel_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/kiwi_robot/joint/rear_left_wheel_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/kiwi_robot/joint/rear_right_wheel_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Converts /cmd_vel (Twist) into the three wheel speed commands above —
    # this was never launched before, so no drive command could ever reach
    # the simulated wheels.
    omni_drive_node = Node(
        package='kiwi_robot',
        executable='omni_drive_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        omni_drive_node,
    ])
