from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera',
            executable='realsense_node',
            name='realsense_camera',
            output='screen',
            remappings=[
                ('/camera/color/image_raw', '/camera/color/image_raw'),
                ('/camera/imu', '/camera/imu')
            ]
        )
    ])