import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

GAZEBO_LAUNCH_FILE = 'gazebo.launch.py' 

WHEEL_JOINTS = ['front_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']


def generate_launch_description():
    pkg_share = get_package_share_directory('kiwi_robot')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', GAZEBO_LAUNCH_FILE)
        )
    )

    # One ROS->GZ bridge per wheel velocity command topic.
    wheel_bridge_args = [
        f'/model/kiwi_robot/joint/{joint}/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double'
        for joint in WHEEL_JOINTS
    ]
    wheel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wheel_cmd_bridge',
        output='screen',
        arguments=wheel_bridge_args,
    )

    omni_drive_node = Node(
        package='kiwi_robot',
        executable='omni_drive_node',
        name='omni_drive_node',
        output='screen',
    )

    slam_params_file = os.path.join(pkg_share, 'launch', 'slam_toolbox_params.yaml')
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
    )

    controller_node = Node(
        package='kiwi_robot',
        executable='navigation_controller.py',
        name='navigation_controller',
        output='screen',
    )

    odometry_node = Node(
        package='kiwi_robot',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
    )
    
    # stagger startup so Gazebo/bridge are up before anything subscribes.
    delayed_wheel_bridge = TimerAction(period=3.0, actions=[wheel_bridge])
    delayed_omni = TimerAction(period=4.0, actions=[omni_drive_node])
    delayed_slam = TimerAction(period=5.0, actions=[slam_node])
    delayed_controller = TimerAction(period=8.0, actions=[controller_node])

    return LaunchDescription([
        gazebo_launch,
        delayed_wheel_bridge,
        delayed_omni,
        delayed_slam,
        delayed_controller,
    ])