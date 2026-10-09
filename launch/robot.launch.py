import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # YDLidar Node (assumes ydlidar_ros2_driver is built and parameter file exists)
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': '/dev/ydlidar',
            'frame_id': 'laser_frame',
            'ignore_array': '',
            'baudrate': 115200,
            'lidar_type': 1,
            'device_type': 0,
            'sample_rate': 3,
            'abnormal_check_count': 4,
            'fixed_resolution': True,
            'reversion': False,
            'inverted': True,
            'auto_reconnect': True,
            'isSingleChannel': False,
            'intensity': False,
            'support_motor_dtr': True,
            'angle_max': 180.0,
            'angle_min': -180.0,
            'range_max': 10.0,
            'range_min': 0.12,
            'frequency': 10.0,
            'invalid_range_is_inf': False
        }]
    )

    # Motor Controller Node
    motor_node = Node(
        package='robot_control',
        executable='motor_controller',
        name='motor_controller',
        output='screen'
    )

    # Encoder Node
    encoder_node = Node(
        package='robot_control',
        executable='encoder_node',
        name='encoder_node',
        output='screen'
    )

    # Obstacle Avoidance Node
    avoidance_node = Node(
        package='robot_control',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen'
    )

    # Static Transform Publisher (base_link -> laser_frame)
    # Allows ROS to know where the LiDAR is relative to the center of the robot
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame'],
    )

    return LaunchDescription([
        tf_node,
        ydlidar_node,
        motor_node,
        encoder_node,
        avoidance_node
    ])
